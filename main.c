/* Includes ----------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "fmt.h"
#include "msg.h"
#include "net/loramac.h"
#include "periph/adc.h"
#include "periph/gpio.h"
#include "periph/hwrng.h"
#include "periph/pm.h"
#include "periph/rtc.h"
#include "scaling.h"
#include "semtech_loramac.h"
#include "thread.h"
#include "xtimer.h"

/* Private typedef ---------------------------------------------------*/

/* Private define ----------------------------------------------------*/
#define SENDER_PRIO (THREAD_PRIORITY_MAIN - 1)
#define RECV_MSG_QUEUE (4U)
#define ON_TIME_S 6
#define RTC_SET_OLD_ALARM_DIFFERENCE 30
#define PM_LOCK_LEVEL (1)

#define ADC_VREF_INT 7
#define VREF_INT_CAL ((uint16_t *) ((uint32_t) 0x1FF80078))

#ifndef DEFAULT_PERIOD_SENDING
#define DEFAULT_PERIOD_SENDING 600
#endif /* DEFAULT_PERIOD_SENDING */

#ifndef DEFAULT_RESOLUTION
#define DEFAULT_RESOLUTION 0
#endif /* DEFAULT_RESOLUTION */

#define INPUT_0 GPIO_PIN(PORT_B, 3)
#define INPUT_1 GPIO_PIN(PORT_B, 4)
#define INPUT_2 GPIO_PIN(PORT_C, 2)
#define INPUT_3 GPIO_PIN(PORT_C, 1)
#define INPUT_4 GPIO_PIN(PORT_B, 5)
#define INPUT_5 GPIO_PIN(PORT_B, 9)
#define INPUT_6 GPIO_PIN(PORT_C, 0)
#define INPUT_7 GPIO_PIN(PORT_C, 13)

#define GET_INPUT_DATA_TIME_S 3
#define GET_INPUT_DATA_SLEEP_TIME_MS 10

#if (GET_INPUT_DATA_TIME_S * MS_PER_SEC / GET_INPUT_DATA_SLEEP_TIME_MS) > UINT16_MAX
#error Check define GET_INPUT_DATA_TIME_S and GET_INPUT_DATA_SLEEP_TIME_MS
#endif

enum RTC_SET_CMD { RTC_SET_NEW_ALARM, RTC_SET_OLD_ALARM };

/* Private macro -----------------------------------------------------*/

/* Private variables -------------------------------------------------*/
static kernel_pid_t sender_pid;
static char sender_stack[THREAD_STACKSIZE_MAIN / 2];
static char recv_stack[THREAD_STACKSIZE_DEFAULT];
static uint8_t deveui[LORAMAC_DEVEUI_LEN];
static uint8_t appeui[LORAMAC_APPEUI_LEN];
static uint8_t appkey[LORAMAC_APPKEY_LEN];
static msg_t recv_queue[RECV_MSG_QUEUE];
semtech_loramac_t loramac;
uint8_t message[50] = {0};
uint32_t sending_interval_s = DEFAULT_PERIOD_SENDING;
uint8_t resolution = DEFAULT_RESOLUTION;
bool interrupt_active = false;

/* Private function prototypes ---------------------------------------*/
static void gpio_irq_cb(void *arg);
static void rtc_cb(void *arg);
static int prepare_next_alarm(enum RTC_SET_CMD cmd);
static void send_message(uint8_t length);
static void *sender(void *arg);
static void *recv(void *arg);
static float get_vcc(void);
static void init_unused_pins(void);
static void disable_gpio_irq(void);
static void enable_gpio_irq(void);
static uint8_t get_input_data(void);
static void set_resolution(uint8_t received_resolution);
static void set_sending_interval(uint16_t received_interval);
static bool get_gpio_state(gpio_t pin);

/* Private functions -------------------------------------------------*/
int main(void)
{
    uint8_t number_join_tries = 0;
    uint8_t loramac_datarate = 0;

    printf("Program: ho_short_circuit_indicator\nFW: 0.3\n");

    /* Init peripherie */
    init_unused_pins();
    adc_init(ADC_LINE(ADC_VREF_INT));

    /* Convert identifiers and application key */
    fmt_hex_bytes(deveui, DEVEUI);
    fmt_hex_bytes(appeui, APPEUI);
    fmt_hex_bytes(appkey, APPKEY);

    /* Initialize the loramac stack */
    semtech_loramac_init(&loramac);
    semtech_loramac_set_deveui(&loramac, deveui);
    semtech_loramac_set_appeui(&loramac, appeui);
    semtech_loramac_set_appkey(&loramac, appkey);

    /* Use a fast datarate, e.g. BW125/SF7 in EU868 */
    semtech_loramac_set_dr(&loramac, LORAMAC_DR_5);

    /* Use ADR */
    semtech_loramac_set_adr(&loramac, true);

    /* Use unconfirmed data mode */
    semtech_loramac_set_tx_mode(&loramac, LORAMAC_TX_UNCNF);

    /* Use port 1 for uplink masseges */
    semtech_loramac_set_tx_port(&loramac, 1);

    /* Start the Over-The-Air Activation (OTAA) procedure to retrieve the
     * generated device address and to get the network and application session
     * keys.
     */
    puts("Starting join procedure");
    while (semtech_loramac_join(&loramac, LORAMAC_JOIN_OTAA) != SEMTECH_LORAMAC_JOIN_SUCCEEDED) {
        puts("Join procedure failed, try in 30s again");
        xtimer_sleep(23);

        /* increase datarate after 3 join tries */
        number_join_tries++;
        loramac_datarate = semtech_loramac_get_dr(&loramac);
        if (number_join_tries > 2 && loramac_datarate > LORAMAC_DR_0) {
            number_join_tries = 0;
            loramac_datarate--;
            semtech_loramac_set_dr(&loramac, loramac_datarate);
        }
    }
    puts("Join procedure succeeded");

    /* start the sender thread */
    sender_pid = thread_create(sender_stack, sizeof(sender_stack), SENDER_PRIO, 0, sender, NULL, "sender");

    /* start the receive thread */
    thread_create(recv_stack, sizeof(recv_stack), THREAD_PRIORITY_MAIN - 1, 0, recv, NULL, "recv thread");

    /* activate gpio irq */
    gpio_init_int(INPUT_0, GPIO_IN_PD, GPIO_RISING, gpio_irq_cb, NULL);
    gpio_init_int(INPUT_1, GPIO_IN_PD, GPIO_RISING, gpio_irq_cb, NULL);
    gpio_init_int(INPUT_2, GPIO_IN_PD, GPIO_RISING, gpio_irq_cb, NULL);
    gpio_init_int(INPUT_3, GPIO_IN_PD, GPIO_RISING, gpio_irq_cb, NULL);
    gpio_init_int(INPUT_4, GPIO_IN_PD, GPIO_RISING, gpio_irq_cb, NULL);
    gpio_init_int(INPUT_5, GPIO_IN_PD, GPIO_RISING, gpio_irq_cb, NULL);
    gpio_init_int(INPUT_6, GPIO_IN_PD, GPIO_RISING, gpio_irq_cb, NULL);
    gpio_init_int(INPUT_7, GPIO_IN_PD, GPIO_RISING, gpio_irq_cb, NULL);

    /* trigger the first send */
    msg_t msg;
    msg_send(&msg, sender_pid);
    return 0;
}

static void gpio_irq_cb(void *arg)
{
    (void) arg;
    msg_t msg;

    //trigger irq only once
    if (interrupt_active == false) {
        interrupt_active = true;
        disable_gpio_irq();
        rtc_clear_alarm();
        pm_block(PM_LOCK_LEVEL);
        msg_send(&msg, sender_pid);
    }
}

static void rtc_cb(void *arg)
{
    (void)arg;
    
    /* block sleep level mode until the next sending cycle has completed */
    pm_block(PM_LOCK_LEVEL);

    msg_t msg;
    msg_send(&msg, sender_pid);
}

static int prepare_next_alarm(enum RTC_SET_CMD cmd)
{
    struct tm time;
    struct tm get_alarm;
    struct tm set_alarm;
    int rc;
    uint8_t tries = 3;

    rtc_get_time(&time);

    /* set new alarm in x seconds or reset old alarm, if not in the next seconds */
    if (cmd == RTC_SET_NEW_ALARM) {
        memcpy(&set_alarm, &time, sizeof(struct tm));
        set_alarm.tm_sec += (sending_interval_s - ON_TIME_S);
        mktime(&set_alarm);
    } else /* cmd == RTC_SET_OLD_ALARM */ {
        rtc_get_alarm(&get_alarm);
        uint32_t ts_time = mktime(&time);
        uint32_t ts_get_alarm = mktime(&get_alarm);

        if ( (ts_time + RTC_SET_OLD_ALARM_DIFFERENCE) < ts_get_alarm) {
            //old alarm
            memcpy(&set_alarm, &get_alarm, sizeof(struct tm));
        } else {
            //new alarm
            memcpy(&set_alarm, &time, sizeof(struct tm));
            set_alarm.tm_sec += (sending_interval_s - ON_TIME_S);
            mktime(&set_alarm);
        }
    }

    do {
        rtc_set_alarm(&set_alarm, rtc_cb, NULL);
        rtc_get_alarm(&get_alarm);
        rc = rtc_tm_compare(&set_alarm, &get_alarm);
        tries--;
    } while ((rc != 0) && (tries != 0));

    if (rc == 0) {
        puts("RTC alarm set");
    } else {
        puts("RTC alarm not set");
    }

    return rc;
}

static void send_message(uint8_t length)
{
    //TODO print data and port

    printf("Sending\n");
    /* Try to send the message */
    uint8_t ret = semtech_loramac_send(&loramac, message, length);
    if (ret != SEMTECH_LORAMAC_TX_DONE) {
        printf("Cannot send message, ret code: %d\n", ret);
        return;
    }
}

static void *sender(void *arg)
{
    (void)arg;

    msg_t msg;
    msg_t msg_queue[8];
    msg_init_queue(msg_queue, 8);

    while (1) {
        msg_receive(&msg);

        if (interrupt_active == true) {
            puts("wake up interrupt");
            message[0] = get_input_data();

            if (message[0] != 0) {
                /* wait 0 - 6,4s, because a short circuit syncronize all sensors and would result in collisions */
                uint8_t buf = 0;
                hwrng_read(&buf, sizeof(buf));
                xtimer_usleep( (buf>>2) * 100000 );

                semtech_loramac_set_tx_mode(&loramac, LORAMAC_TX_CNF);
                semtech_loramac_set_tx_port(&loramac, 3);  //TODO Port enum
                send_message(1);
            }
        } else {
            puts("wake up rtc");
            semtech_loramac_set_tx_mode(&loramac, LORAMAC_TX_UNCNF);
            if (resolution == 0) {
                semtech_loramac_set_tx_port(&loramac, 1);  //TODO Port enum
                uint8_t vbat = (uint8_t)scaling_float(get_vcc(), 2.0, 4.0, 0.0, 255.0, LIMIT_OUTPUT);
                message[0] = (uint8_t)vbat;
                send_message(1);
            } else {
                semtech_loramac_set_tx_port(&loramac, 2);  //TODO Port enum
                uint16_t vbat = (uint16_t)scaling_float(get_vcc(), 2.0, 4.0, 0.0, 65535.0, LIMIT_OUTPUT);
                message[0] = (uint8_t)(vbat >> 8);
                message[1] = (uint8_t)(vbat);
                send_message(2);
            }
        }

        /* Schedule the next wake-up alarm */
        int rc_rtc = 0;

        if (interrupt_active == true) {
            interrupt_active = false;
            rc_rtc = prepare_next_alarm(RTC_SET_OLD_ALARM);
        } else {
            rc_rtc = prepare_next_alarm(RTC_SET_NEW_ALARM);
        }

        if (rc_rtc == 0) {
            /* going to deep sleep */
            puts("Going to sleep");
            enable_gpio_irq();
            pm_unblock(PM_LOCK_LEVEL);
        } else {
            /* rtc can not set, try reboot */
            pm_reboot();
        }
    }

    /* this should never be reached */
    return NULL;
}

static void *recv(void *arg) {
    msg_init_queue(recv_queue, RECV_MSG_QUEUE);

    (void)arg;

    while (1) {
        /* blocks until some data is received */
        switch (semtech_loramac_recv(&loramac)) {
            case SEMTECH_LORAMAC_RX_DATA:
                loramac.rx_data.payload[loramac.rx_data.payload_len] = 0;
                printf("Data received: %s, port: %d\n", (char *)loramac.rx_data.payload, loramac.rx_data.port);

                /* process received data */
                switch (loramac.rx_data.port) {
                    case 1:
                        set_resolution(loramac.rx_data.payload[0]);
                        break;
                    
                    case 2:
                        set_sending_interval((loramac.rx_data.payload[0] << 8) + loramac.rx_data.payload[1]);
                        break;
                    
                    case 3: /* System reboot */
                        if (loramac.rx_data.payload[0] == 1) {
                            pm_reboot();
                        }
                        break;
                    
                    default:
                        break;
                }
                break;

            case SEMTECH_LORAMAC_RX_CONFIRMED:
                puts("Received ACK from network");
                break;

            default:
                break;
        }
    }
    return NULL;
}

static float get_vcc(void) {
    uint16_t *vref_int_cal = VREF_INT_CAL;
    float vbat = 3.0 * *vref_int_cal / adc_sample(ADC_VREF_INT, ADC_RES_12BIT); //TODO catch failure of adc_sample 
    return vbat;
}

static void init_unused_pins(void) {
    gpio_t unused_pins[] = {
        //GPIO_PIN(PORT_A, 0),
        GPIO_PIN(PORT_A, 1),
        //GPIO_PIN(PORT_A, 2),
        GPIO_PIN(PORT_A, 3),
        //GPIO_PIN(PORT_A, 4), NSS
        //GPIO_PIN(PORT_A, 5), SCK
        //GPIO_PIN(PORT_A, 6), MISO
        //GPIO_PIN(PORT_A, 7), MOSI
        GPIO_PIN(PORT_A, 8),
        GPIO_PIN(PORT_A, 9),
        GPIO_PIN(PORT_A, 10),
        GPIO_PIN(PORT_A, 11),
        GPIO_PIN(PORT_A, 12),
        GPIO_PIN(PORT_A, 13),
        GPIO_PIN(PORT_A, 14),
        GPIO_PIN(PORT_A, 15),
        GPIO_PIN(PORT_B, 0),
        GPIO_PIN(PORT_B, 1),
        GPIO_PIN(PORT_B, 2),
        //GPIO_PIN(PORT_B, 3),
        GPIO_PIN(PORT_B, 4),
        //GPIO_PIN(PORT_B, 5),
        GPIO_PIN(PORT_B, 6),
        //GPIO_PIN(PORT_B, 7),
        GPIO_PIN(PORT_B, 8),
        //GPIO_PIN(PORT_B, 9),
        GPIO_PIN(PORT_B, 10),
        GPIO_PIN(PORT_B, 11),
        GPIO_PIN(PORT_B, 12),
        GPIO_PIN(PORT_B, 13),
        GPIO_PIN(PORT_B, 14),
        GPIO_PIN(PORT_B, 15),
        GPIO_PIN(PORT_C, 0),
        //GPIO_PIN(PORT_C, 1),
        GPIO_PIN(PORT_C, 2),
        //GPIO_PIN(PORT_C, 3),
        GPIO_PIN(PORT_C, 4),
        //GPIO_PIN(PORT_C, 5),
        //GPIO_PIN(PORT_C, 6), DIO0
        //GPIO_PIN(PORT_C, 7), DIO1
        //GPIO_PIN(PORT_C, 8), DIO2
        //GPIO_PIN(PORT_C, 9), RESET
        //GPIO_PIN(PORT_C, 10),
        GPIO_PIN(PORT_C, 11),
        GPIO_PIN(PORT_C, 12),
        //GPIO_PIN(PORT_C, 13),
        //GPIO_PIN(PORT_C, 14), RTC crystal
        //GPIO_PIN(PORT_C, 15), RTC crystal
        //GPIO_PIN(PORT_D, 2), MOSFET
        GPIO_PIN(PORT_H, 0),
        GPIO_PIN(PORT_H, 1)
    };
    for (uint8_t i = 0; i < (sizeof(unused_pins) / sizeof(gpio_t)); i++) {
        gpio_init(unused_pins[i], GPIO_IN_PD);
    }
}

static void disable_gpio_irq(void)
{
    gpio_irq_disable(INPUT_0);
    gpio_irq_disable(INPUT_1);
    gpio_irq_disable(INPUT_2);
    gpio_irq_disable(INPUT_3);
    gpio_irq_disable(INPUT_4);
    gpio_irq_disable(INPUT_5);
    gpio_irq_disable(INPUT_6);
    gpio_irq_disable(INPUT_7);
}

static void enable_gpio_irq(void)
{
    gpio_irq_enable(INPUT_0);
    gpio_irq_enable(INPUT_1);
    gpio_irq_enable(INPUT_2);
    gpio_irq_enable(INPUT_3);
    gpio_irq_enable(INPUT_4);
    gpio_irq_enable(INPUT_5);
    gpio_irq_enable(INPUT_6);
    gpio_irq_enable(INPUT_7);
}

static uint8_t get_input_data(void)
{
    uint8_t ct0 = 0xFF;
    uint8_t ct1 = 0xFF;
    uint8_t data = 0;
    uint8_t j = 0;
    uint8_t port = 0;
    uint8_t key_state = 0;
    puts("get input data:");

    for (uint16_t i = 0; i < (GET_INPUT_DATA_TIME_S * MS_PER_SEC / GET_INPUT_DATA_SLEEP_TIME_MS); i++) {
        port = (
            get_gpio_state(INPUT_7) << 7 |
            get_gpio_state(INPUT_6) << 6 |
            get_gpio_state(INPUT_5) << 5 |
            get_gpio_state(INPUT_4) << 4 |
            get_gpio_state(INPUT_3) << 3 |
            get_gpio_state(INPUT_2) << 2 |
            get_gpio_state(INPUT_1) << 1 |
            get_gpio_state(INPUT_0)
        );

        /* debouncing by Peter Dannegger */
        j = key_state ^ port;             // key changed ?
        ct0 = ~( ct0 & j );               // reset or count ct0
        ct1 = ct0 ^ (ct1 & j);            // reset or count ct1
        j &= ct0 & ct1;                   // count until roll over ?
        key_state ^= j;                   // then toggle debounced state
        data |= key_state & j;            // 0->1: key press detect

        xtimer_usleep(GET_INPUT_DATA_SLEEP_TIME_MS * US_PER_MS);
    }

    printf("%d\n", data);

    return data;
}

static void set_resolution(uint8_t received_resolution)
{
    if (received_resolution <= 1) {
        resolution = received_resolution;
    }
}

static void set_sending_interval(uint16_t received_interval)
{
    if (received_interval >= 3) {
        sending_interval_s = received_interval * 10;
    }
}

static bool get_gpio_state(gpio_t pin)
{
    if(gpio_read(pin)) {
        return true;
    } else {
        return false;
    }
}
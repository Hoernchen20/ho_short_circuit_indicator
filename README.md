# ho_short_circuit_indicator
clone this repository: `git clone --recurse-submodules https://github.com/Hoernchen20/ho_short_circuit_indicator.git`
update submodules: `git submodule update`

apply git-diff to RIOT: `cd RIOT; git apply ../git.diff`

Change the keys in `lorawan.credentials.example` and rename to `lorawan.credentials`.

build: `make`

binary is located in `bin/nucleo-073rz`
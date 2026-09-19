# plant-abi — the engine's C ABI: the shim and its two self-checks.
#
#   make cabi-build   # build libeng_shim.dylib on MuJoCo's C API (prerequisite of every target
#                     # that steps a plant; build.rs fails without it)
#   make cabi-check   # the ABI checks that need no Rust
#
# Requires: clang >= 15, python3 + unzip (cabi/build_mj.sh fetches MuJoCo's prebuilt library on the
# first run). MJ and BUILD are the same cache a consumer's build.rs reads; MODELS is control-model's
# data directory, and an argument because this crate holds the ABI and not the models.

MJ     ?= $(HOME)/.cache/simu/mj
BUILD  ?= $(HOME)/.cache/simu/mj_build
MODELS ?= ../control-model/models

.PHONY: cabi-build cabi-check

cabi-build:
	cabi/build_mj.sh $(MJ) $(BUILD)

# The ABI checks that need no Rust: mj_shim_smoke drives the eng_* calls the plants do; mj_model_load
# loads every model the two products have and checks the DOF count their own tests assert.
cabi-check: cabi-build
	cc -O2 -Icabi -F$(MJ) -L$(BUILD)/bin -o $(BUILD)/mj_shim_smoke cabi/mj_shim_smoke.c \
	   -framework mujoco -leng_shim -Wl,-rpath,$(MJ) -Wl,-rpath,$(BUILD)/bin
	$(BUILD)/mj_shim_smoke $(MODELS)
	cc -O2 -Icabi -F$(MJ) -L$(BUILD)/bin -o $(BUILD)/mj_model_load cabi/mj_model_load.c \
	   -framework mujoco -leng_shim -Wl,-rpath,$(MJ) -Wl,-rpath,$(BUILD)/bin
	$(BUILD)/mj_model_load $(MODELS)

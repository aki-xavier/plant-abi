# plant-abi — the physics engine's C ABI

A project of its own: the one place the ABI between a Rust control core and a
physics engine is written down. MIT-licensed (see `LICENSE`).

## What is here

```text
cabi/eng_shim.h                      the contract: 27 functions and what each promises
cabi/mj_shim.c                       one implementation of it (MuJoCo), ~2200 lines of C
cabi/build_mj.sh                     builds libeng_shim.dylib against MuJoCo's framework
cabi/mj_model_load.c                 self-check: loads every model and checks its DOF count
cabi/mj_shim_smoke.c                 self-check: drives every eng_* entry point the header declares
src/ffi.rs                           the contract as Rust: one `extern "C"` per header function
src/handles.rs                       Engine / Scene / Robot — the ABI's voidptr fields, owned in Rust
build.rs                             links the shim, and adds the two rpaths its consumers need
```

Two Rust modules and nothing else, and **no dependencies at all** — not even `control-math`.

## Why it is a project of its own

The ABI is the one thing that may not be copied per holder.

- **A copy of `eng_shim.h` per repository is two ABIs, and two ABIs drift.** The
  drift is not a compile error in either repository; it is a wrong number at
  runtime in whichever one was edited second.
- **A crate that declares the engine's functions itself cannot be built without
  the engine.** Keeping the declarations here means anything that depends on this
  gets the engine, and anything that does not stays pure Rust.

`handles.rs` is where the ABI's lifetimes live: a `Robot` cannot outlive its
`Scene`, a `Scene` cannot outlive its `Engine`, and the drop order is the ABI's
own. The contract a controller programs against is not here — it is
[`control-base`](../control-base)'s `Plant`, which names no engine at all. This
crate is the boundary between those two facts.

## Building

```sh
make cabi-build               # writes ~/.cache/simu/mj_build/bin/libeng_shim.dylib
make cabi-check               # the two C self-checks, against ../control-model/models
```

`cabi-build` first, for every target: this crate IS the ABI, so there is no
feature to gate the link on and `build.rs` fails loudly if the shim is missing.
The shim's location and the `SIMU_ENGINE_DIR` / `SIMU_MJ_DIR` names are kept as
they were rather than renamed — they are a build ritual a working tree already
has, not this crate's vocabulary.

The checks are the C self-checks rather than `mbx test`, and they are about
what the ABI does: `mj_model_load` pins the DOF count of every model, and
`mj_shim_smoke` drives every `eng_*` entry point the header declares — context,
scene, ground, wall, attach (both attach paths), state, step, the per-link
contact tables, the report mode a caller states, the policy writers, the render,
and the failures that must arrive as a message rather than as a plausible zero.
What this crate does NOT test is its own boundary — no dependency set, no
vocabulary and no C-header-to-Rust agreement check. A boundary a build keeps is
the dependency graph's job: the declarations have one home here, so the drift
those checks would look for has nowhere to happen.

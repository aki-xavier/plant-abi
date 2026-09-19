# plant-abi — the physics engine's C ABI

A project of its own: the one place the ABI between a Rust control core and a
physics engine is written down. MIT-licensed (see `LICENSE`).

## What is here

```text
cabi/eng_shim.h                      the contract: 28 functions and what each promises
cabi/mj_shim.c                       one implementation of it (MuJoCo), ~2200 lines of C
cabi/build_mj.sh                     builds libeng_shim.dylib against MuJoCo's framework
cabi/mj_model_load.c                 self-check: loads every model and checks its DOF count
cabi/mj_shim_smoke.c                 self-check: drives the eng_* calls the plants make
src/ffi.rs                           the contract as Rust: one `extern "C"` per header function
src/handles.rs                       Engine / Scene / Robot — the ABI's voidptr fields, owned in Rust
build.rs                             links the shim, and adds the two rpaths its consumers need
tests/charter.rs                     four claims about the boundary, checked rather than promised
```

Two Rust modules and nothing else, and **no dependencies at all** — not even `control-math`.

## Why it is a project of its own

The ABI is the one thing two products that share an engine may not each hold.

- **A copy of `eng_shim.h` per repository is two ABIs, and two ABIs drift.** The
  drift is not a compile error in either repository; it is a wrong number at
  runtime in whichever one was edited second.
- **A crate that declares the engine's functions itself cannot be built without
  the engine.** Keeping the declarations here means a product either depends on
  this and gets the engine, or does not and stays pure Rust.

`handles.rs` is where the ABI's lifetimes live: a `Robot` cannot outlive its
`Scene`, a `Scene` cannot outlive its `Engine`, and the drop order is the ABI's
own. The contract a controller programs against is not here — it is
[`control-base`](../control-base)'s `Plant`, which names no engine at all. This
crate is the boundary between those two facts.

## Building

```sh
make -C ../simu cabi-build    # writes ~/.cache/simu/mj_build/bin/libeng_shim.dylib
cargo test                    # the four charter claims
```

`cabi-build` first, for every target: this crate IS the ABI, so there is no
feature to gate the link on and `build.rs` fails loudly if the shim is missing.
The shim's location and the `SIMU_ENGINE_DIR` / `SIMU_MJ_DIR` names are kept as
they were rather than renamed — they are a build ritual a working tree already
has, not this crate's vocabulary.

`cargo test` needs the shim on disk but not a working engine: the charter tests
read the sources, and the header/Rust agreement check is arithmetic about names.

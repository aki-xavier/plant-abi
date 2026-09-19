// plant-abi — the physics engine's C ABI, as a project of its own.
//
// TWO MODULES AND NOTHING ELSE:
//   ffi      the eng_* declarations, one per function in cabi/eng_shim.h — the whole contract between
//            the Rust control core and the engine that stands behind the ABI.
//   handles  Engine / Scene / Robot: the ABI's raw voidptr fields owned in Rust, with the ABI's own
//            order enforced by the lifetimes (a Robot cannot outlive its Scene, a Scene cannot outlive
//            its Engine) and every failure funnelled through `last_error`.
//
// WHY IT IS A PROJECT OF ITS OWN: the ABI is the one thing that may not be copied per holder. Two
// copies of `eng_shim.h` are two ABIs, and two ABIs drift — and a crate that declares the engine's
// functions itself is a crate that cannot be built without the engine. Here the C shim, its Rust
// declarations and its build ritual live in one place; anything that depends on this gets the engine,
// and anything that does not stays pure Rust.
//
// WHAT IT IS NOT: it names no model, no plant and no control law — nothing above it at all. The
// `Plant` contract a controller programs against is ../control-base's; the plants that drive these
// handles belong to whoever has them, and this crate learns about none of them. It is the boundary
// itself.

pub mod ffi;
pub mod handles;

// plant-abi — the physics engine's C ABI, as a project of its own.
//
// TWO MODULES AND NOTHING ELSE:
//   ffi      the eng_* declarations, one per function in cabi/eng_shim.h — the whole contract between
//            the Rust control core and the engine that stands behind the ABI.
//   handles  Engine / Scene / Robot: the ABI's raw voidptr fields owned in Rust, with the ABI's own
//            order enforced by the lifetimes (a Robot cannot outlive its Scene, a Scene cannot outlive
//            its Engine) and every failure funnelled through `last_error`.
//
// WHY IT IS A PROJECT OF ITS OWN: the ABI is the one thing two products that share an engine may not
// each hold. A copy of `eng_shim.h` per repository is two ABIs, and two ABIs drift — and a crate that
// declares the engine's functions itself is a crate that cannot be built without the engine. Here the
// C shim, its Rust declarations and its build ritual live in one place; a product depends on this and
// gets the engine, or does not and gets a pure-Rust plant.
//
// WHAT IT IS NOT: it names no model, no plant and no control law. The `Plant` contract a controller
// programs against is ../control-base's, and the plants that drive these handles belong to the
// products that have them (simu's CEnginePlant, its BipedPlant). This crate is the boundary itself.

pub mod ffi;
pub mod handles;

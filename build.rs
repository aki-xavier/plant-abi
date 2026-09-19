// build.rs — link the MuJoCo C ABI shim. This package IS the ABI, so there is no feature to gate it on:
// depending on `plant-abi` means taking the engine, and `make cabi-build` is a prerequisite of every
// target that does — run it first.
//
// SIMU_ENGINE_DIR (default ~/.cache/simu/mj_build/bin) mirrors simu's Makefile BUILD ?=, SIMU_MJ_DIR
// (default ~/.cache/simu/mj) its MJ ?=. The names and the cache path are kept rather than renamed: they
// are a build ritual a working tree already has, not this crate's vocabulary, and re-spelling them
// would invalidate every existing shim. The shim carries the rpath to MuJoCo's own framework; the two
// rpaths below are the ones this crate must add for its consumers' binaries.

use std::env;
use std::path::Path;

fn main() {
    let home = env::var("HOME").expect("HOME is not set");
    let engine_dir =
        env::var("SIMU_ENGINE_DIR").unwrap_or_else(|_| format!("{home}/.cache/simu/mj_build/bin"));
    let mj_dir = env::var("SIMU_MJ_DIR").unwrap_or_else(|_| format!("{home}/.cache/simu/mj"));

    let dylib = Path::new(&engine_dir).join("libeng_shim.dylib");
    if !dylib.exists() {
        panic!(
            "libeng_shim.dylib not found at {}: build it with `make cabi-build` \
             (or point SIMU_ENGINE_DIR at the directory holding it)",
            dylib.display()
        );
    }

    println!("cargo:rustc-link-search=native={engine_dir}");
    println!("cargo:rustc-link-lib=dylib=eng_shim");
    println!("cargo:rustc-link-arg=-Wl,-rpath,{engine_dir}");
    println!("cargo:rustc-link-arg=-Wl,-rpath,{mj_dir}");

    println!("cargo:rerun-if-env-changed=SIMU_ENGINE_DIR");
    println!("cargo:rerun-if-env-changed=SIMU_MJ_DIR");
    println!("cargo:rerun-if-changed=build.rs");
}

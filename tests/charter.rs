// charter.rs — this crate's own boundary, as a check rather than a promise. What keeps an ABI crate
// honest is not that it is small: it is that it stays BELOW everything. The moment one of its files
// names a model, a plant or a law, the boundary has moved up into it, and a product can no longer take
// the engine without taking its neighbours' business along. Four claims, read off the sources:
//
//   1. every import is std or this crate's own — there is no dependency here to be had;
//   2. no source names a model, a plant or a control law in CODE (the lists below are the only place
//      those words are written down, because a check has to name what it forbids);
//   3. ffi.rs and cabi/eng_shim.h declare the SAME functions: the header is the contract and the Rust
//      is its transcription, so a function on one side and not the other is a hole in the ABI (this
//      check found one on its first run — eng_robot_set_contact_report was implemented in mj_shim.c
//      and called by a plant, but the header never declared it);
//   4. the manifest has no dependencies and DOES have a build script — this is the one crate whose job
//      is to link something.

use std::collections::BTreeSet;
use std::fs;
use std::path::PathBuf;

fn root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

// sources collects every `.rs` under src/ as (name, text): a walk that missed a subdirectory would
// leave the charter's escape hatch open.
fn sources() -> Vec<(String, String)> {
    let mut out = Vec::new();
    let dir = root().join("src");
    let entries = fs::read_dir(&dir).unwrap_or_else(|_| panic!("cannot read {}", dir.display()));
    let mut paths: Vec<PathBuf> = entries.filter_map(|e| e.ok()).map(|e| e.path()).collect();
    paths.sort();
    for p in paths {
        let name = p.file_name().unwrap().to_string_lossy().to_string();
        if name.ends_with(".rs") {
            let text = fs::read_to_string(&p).unwrap_or_else(|_| panic!("cannot read {name}"));
            out.push((name, text));
        }
    }
    out
}

// code_of drops whole-line comments: the check is about what the sources do, and the comments are
// prose about it rather than code.
fn code_of(text: &str) -> String {
    let mut out: Vec<&str> = Vec::new();
    for line in text.lines() {
        if !line.trim().starts_with("//") {
            out.push(line);
        }
    }
    out.join("\n")
}

/// The module list is the whole crate — `ffi` and `handles` — and every import inside it is std or
/// this crate's own. A third module is a second job; a third import is a dependency.
#[test]
fn every_import_is_std_or_our_own() {
    let files = sources();
    assert_eq!(
        files.iter().map(|(n, _)| n.as_str()).collect::<Vec<_>>(),
        vec!["ffi.rs", "handles.rs", "lib.rs"],
        "the module list moved: this crate is the ABI and the handles over it, and nothing else"
    );
    for (name, text) in &files {
        for line in code_of(text).lines() {
            let t = line.trim();
            if !t.starts_with("use ") {
                continue;
            }
            assert!(
                t.starts_with("use std::")
                    || t.starts_with("use crate::")
                    || t.starts_with("use self::")
                    || t.starts_with("use super::"),
                "{name} imports outside std and this crate: {t}"
            );
        }
    }
}

/// No model, no plant, no control law. The lists are the names that would mean the boundary had moved:
/// a model layer's types, the concrete plants that drive these handles, and the laws and layer programs
/// above them — none of which this crate may know it has.
#[test]
fn nothing_here_names_a_model_a_plant_or_a_law() {
    let forbidden: [(&str, &[&str]); 3] = [
        (
            "a model type",
            &[
                "BodyTree",
                "TreeDynamicsModel",
                "PgaFk",
                "PgaDynamicsModel",
                "UrdfChain",
                "MjcfModel",
            ],
        ),
        (
            "a concrete plant",
            &[
                "CEnginePlant",
                "BipedPlant",
                "NominalView",
                "FakeEnginePlant",
            ],
        ),
        (
            "a control law",
            &[
                "PlaneTaskLoop",
                "PlaneDesign",
                "StandingLoop",
                "GaitRuntime",
                "SpinalReflex",
                "ReflexLayer",
                "BehaviorArbiter",
                "Predictor",
                "Keepout",
                "Stepper",
            ],
        ),
    ];
    for (name, text) in &sources() {
        let code = code_of(text);
        for (what, names) in forbidden {
            for bad in names {
                assert!(
                    !code.contains(bad),
                    "{name} names {what} ({bad}) in code: this crate is the boundary, and the things \
                     above it must not be able to reach into it"
                );
            }
        }
    }
    // and the lists are anchored: the ABI really is declared here, so the checks above are not passing
    // by looking at files that hold nothing
    let ffi = sources()
        .into_iter()
        .find(|(n, _)| n == "ffi.rs")
        .expect("src/ffi.rs is gone: the ABI is not declared in this crate");
    assert!(
        ffi.1.contains("pub fn eng_robot_step") && ffi.1.contains("pub type Handle"),
        "src/ffi.rs no longer declares the ABI"
    );
}

/// The contract and its transcription are the same set. The header is what a C shim implements; ffi.rs
/// is what a Rust plant calls. A function on one side and not the other builds and links and then
/// surprises someone, which is the one failure a boundary crate can have that nothing else catches.
#[test]
fn the_rust_declarations_and_the_header_agree() {
    let header = fs::read_to_string(root().join("cabi").join("eng_shim.h")).expect("eng_shim.h");
    let rust = fs::read_to_string(root().join("src").join("ffi.rs")).expect("src/ffi.rs");
    // DECLARATIONS ONLY: the header's are prototypes (a type, then the name), ffi.rs's are `pub fn`.
    // Scanning every occurrence instead would read the prose in the two files' comments, which is not
    // the contract and does not have to agree.
    let ident = |rest: &str| -> String {
        let end = rest
            .find(|c: char| !(c.is_ascii_alphanumeric() || c == '_'))
            .unwrap_or(rest.len());
        rest[..end].to_string()
    };
    let header_names = |text: &str| -> BTreeSet<String> {
        let mut out = BTreeSet::new();
        for line in text.lines() {
            let t = line.trim();
            let proto = t.starts_with("int ")
                || t.starts_with("void ")
                || t.starts_with("void* ")
                || t.starts_with("const char* ");
            if proto {
                if let Some(start) = t.find("eng_") {
                    out.insert(ident(&t[start..]));
                }
            }
        }
        out
    };
    let rust_names = |text: &str| -> BTreeSet<String> {
        let mut out = BTreeSet::new();
        for line in text.lines() {
            if let Some(rest) = line.trim().strip_prefix("pub fn ") {
                if rest.starts_with("eng_") {
                    out.insert(ident(rest));
                }
            }
        }
        out
    };
    let h = header_names(&header);
    let r = rust_names(&rust);
    let missing_in_rust: Vec<&String> = h.difference(&r).collect();
    let missing_in_header: Vec<&String> = r.difference(&h).collect();
    assert!(
        missing_in_rust.is_empty(),
        "the header declares, and ffi.rs does not: {missing_in_rust:?}"
    );
    assert!(
        missing_in_header.is_empty(),
        "ffi.rs declares, and the header does not: {missing_in_header:?}"
    );
    assert!(
        h.len() >= 25,
        "the ABI shrank to {} names: this check would pass on nothing",
        h.len()
    );
}

/// The dependency set is the charter's own: none, and a build script. A boundary that needs a library
/// above it is not a boundary, and one without a build script has nothing to link.
#[test]
fn the_manifest_has_no_dependencies_and_a_build_script() {
    let manifest = fs::read_to_string(root().join("Cargo.toml")).expect("Cargo.toml");
    let mut in_deps = false;
    let mut deps: Vec<String> = Vec::new();
    for line in manifest.lines() {
        let t = line.trim();
        if t.starts_with('[') {
            in_deps = t == "[dependencies]";
            continue;
        }
        if in_deps && !t.is_empty() && !t.starts_with('#') {
            deps.push(t.to_string());
        }
    }
    assert!(
        deps.is_empty(),
        "the ABI crate has a dependency: {deps:?}. It is the bottom of the stack."
    );
    assert!(
        root().join("build.rs").exists(),
        "no build script: this is the crate that links the shim"
    );
    assert!(
        manifest.contains("links = \"eng_shim\""),
        "the manifest does not claim the eng_shim native library, so two packages could claim it"
    );
}

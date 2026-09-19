#!/bin/bash
# Builds the MuJoCo-backed eng_shim dylib: the eng_* C ABI the control core
# binds (`cabi/eng_shim.h`), implemented on MuJoCo's C API. This is the only
# engine build in the tree — build.rs points at this script's output directory.
#
# Usage:  cabi/build_mj.sh [MUJOCO_ROOT] [BUILD]
#   MUJOCO_ROOT  a directory holding mujoco.framework (see the note below)
#   BUILD        output root; the dylib lands in $BUILD/bin
#
# MuJoCo is fetched as the official prebuilt library. The headers and the
# dylib come from the published release; the dylib's install name points into
# mujoco.framework, so the framework layout is assembled here rather than
# linking a bare .dylib (which dyld then refuses to load).
set -e

MJ_ROOT=${1:-$HOME/.cache/simu/mj}
BUILD=${2:-$HOME/.cache/simu/mj_build}
MJ_VERSION=${MJ_VERSION:-3.13.0}
ROOT="$(cd "$(dirname "$0")/.." && pwd)"

# MuJoCo is the official prebuilt library, fetched into $MJ_ROOT when it is not
# already there. It is not vendored into the repository: the engine build this
# replaces sparse-cloned its engine at build time too, and the same reasoning
# applies here. The dylib's install name points into mujoco.framework, so the
# framework layout is assembled rather than linking a bare .dylib, which dyld
# then refuses to load.
if [ ! -f "$MJ_ROOT/mujoco.framework/Versions/A/Headers/mujoco.h" ]; then
  echo "== fetching MuJoCo $MJ_VERSION into $MJ_ROOT =="
  mkdir -p "$MJ_ROOT"
  WHL_DIR="$MJ_ROOT/.whl"
  mkdir -p "$WHL_DIR"
  python3 -m pip download --no-deps --only-binary :all: \
    --python-version 310 --platform macosx_11_0_arm64 \
    "mujoco==$MJ_VERSION" -d "$WHL_DIR" >/dev/null
  unzip -q -o "$WHL_DIR"/mujoco-*.whl -d "$WHL_DIR/x"
  V="$(ls -d "$MJ_ROOT"/mujoco.framework/Versions/A 2>/dev/null || echo "$MJ_ROOT/mujoco.framework/Versions/A")"
  mkdir -p "$V/Headers" "$V/Resources"
  cp "$WHL_DIR"/x/mujoco/libmujoco.*.dylib "$V/"
  cp -R "$WHL_DIR"/x/mujoco/include/mujoco/. "$V/Headers/"
  ln -sf A "$MJ_ROOT/mujoco.framework/Versions/Current"
  ln -sf Versions/Current/Headers "$MJ_ROOT/mujoco.framework/Headers"
  ln -sf Versions/Current/Resources "$MJ_ROOT/mujoco.framework/Resources"
  DYLIB="$(cd "$V" && ls libmujoco.*.dylib | head -1)"
  ln -sf "Versions/Current/$DYLIB" "$MJ_ROOT/mujoco.framework/mujoco"
  cat > "$V/Resources/Info.plist" <<'PLIST'
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0"><dict>
  <key>CFBundleExecutable</key><string>mujoco</string>
  <key>CFBundleIdentifier</key><string>org.mujoco</string>
  <key>CFBundleName</key><string>mujoco</string>
  <key>CFBundlePackageType</key><string>FMWK</string>
</dict></plist>
PLIST
  echo "== MuJoCo laid out at $MJ_ROOT/mujoco.framework =="
fi

mkdir -p "$BUILD/bin"
cc -O2 -fPIC -Wall -Wextra \
  -I"$ROOT/cpp" \
  -F"$MJ_ROOT" \
  -arch arm64 -arch x86_64 \
  -shared -o "$BUILD/bin/libeng_shim.dylib" "$ROOT/cabi/mj_shim.c" \
  -framework mujoco \
  -framework OpenGL \
  -Wl,-rpath,"$MJ_ROOT"

echo "built $BUILD/bin/libeng_shim.dylib"
echo "build.rs links this path directly (-L $BUILD/bin)"

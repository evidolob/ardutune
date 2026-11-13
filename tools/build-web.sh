#!/usr/bin/env bash
# Builds the web app into dist/web: ArduTune's core as WebAssembly, the
# desktop app's UI, and the web stand-ins for Tauri. Needs the
# wasm32-unknown-unknown target and wasm-bindgen-cli matching the
# wasm-bindgen version in crates/web/Cargo.toml.
#   tools/build-web.sh           # release build
#   python3 -m http.server -d dist/web 8080
set -euo pipefail
cd "$(dirname "$0")/.."
out=dist/web
rm -rf "$out"
mkdir -p "$out/pkg" "$out/icons"

cargo build --release --locked -p ardutune-web --target wasm32-unknown-unknown
wasm-bindgen --target web --no-typescript --out-dir "$out/pkg" target/wasm32-unknown-unknown/release/ardutune_web.wasm

cp crates/app/ui/app.js crates/app/ui/style.css "$out/"
cp crates/web/www/*.html crates/web/www/*.js crates/web/www/manifest.webmanifest "$out/"
cp crates/app/icons/128x128.png crates/app/icons/128x128@2x.png crates/app/icons/icon.png "$out/icons/"

# A new cache name for each build, so installed copies update.
version="$(grep -m1 '^version' Cargo.toml | cut -d'"' -f2)-$(git rev-parse --short HEAD 2>/dev/null || date +%s)"
sed -i.bak "s/const VERSION = \"dev\";/const VERSION = \"$version\";/" "$out/sw.js" && rm "$out/sw.js.bak"
echo "Built $out ($version)"

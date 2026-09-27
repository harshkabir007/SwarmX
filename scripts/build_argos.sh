#!/usr/bin/env bash
# Build ARGoS3 from source (no sudo; installs to ~/.local/argos3) and the SwarmX ARGoS plugins.
# Requires the ARGoS build dependencies from scripts/install_deps.sh.
# Alternative without installing anything: docker build -f docker/Dockerfile.argos -t swarmx-argos docker
set -eo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PREFIX="${ARGOS_PREFIX:-$HOME/.local/argos3}"
SRC="${ARGOS_SRC:-$HOME/.cache/argos3-src}"
[ -d "$SRC" ] || git clone --depth 1 https://github.com/ilpincy/argos3.git "$SRC"
mkdir -p "$SRC/build_simulator" && cd "$SRC/build_simulator"
cmake ../src -DCMAKE_BUILD_TYPE=Release -DARGOS_DOCUMENTATION=OFF -DARGOS_BUILD_FOR=simulator \
      -DARGOS_WITH_LUA=ON -DCMAKE_INSTALL_PREFIX="$PREFIX"
make -j"$(nproc)" && make install
mkdir -p "$ROOT/argos/build" && cd "$ROOT/argos/build"
cmake .. -DSWARMX_ARGOS_PREFIX="$PREFIX" && make -j"$(nproc)"
echo
echo "export PATH=$PREFIX/bin:\$PATH LD_LIBRARY_PATH=$PREFIX/lib/argos3:\$LD_LIBRARY_PATH"

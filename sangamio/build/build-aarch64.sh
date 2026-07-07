#!/bin/sh
# Cross-build the SangamIO daemon as a static aarch64 (musl) binary for the
# Dreame W10 robot, via Docker. Output: sangamio/build/out/sangamio.
#
# Assembles a minimal Docker context (the daemon crate + its path-dep proto
# crate, excluding target/) so the whole VacuumTiger tree is not uploaded.
set -eu

HERE=$(cd "$(dirname "$0")" && pwd)
SANGAMIO=$(cd "$HERE/.." && pwd)
ROOT=$(cd "$SANGAMIO/.." && pwd)

CTX=$(mktemp -d)
trap 'rm -rf "$CTX"' EXIT

echo ">> staging build context"
rsync -a --exclude 'target/' --exclude '__pycache__/' --exclude 'maps/' \
    "$SANGAMIO/" "$CTX/sangamio/"
mkdir -p "$CTX/dreame-w10"
rsync -a --exclude 'target/' "$ROOT/dreame-w10/proto/" "$CTX/dreame-w10/proto/"
cp "$HERE/Dockerfile" "$CTX/Dockerfile"

echo ">> docker build (aarch64 static musl)"
docker build -f "$CTX/Dockerfile" -t sangamio-aarch64-build "$CTX"

mkdir -p "$HERE/out"
cid=$(docker create sangamio-aarch64-build)
docker cp "$cid:/out/sangamio" "$HERE/out/sangamio"
docker rm "$cid" >/dev/null

echo ">> built $HERE/out/sangamio"
file "$HERE/out/sangamio" 2>/dev/null || true

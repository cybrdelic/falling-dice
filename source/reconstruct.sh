#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
cat "$ROOT"/archive-parts/part*.b64 | base64 --decode > "$ROOT/falling-dice-maintained-source.tar.xz"
echo "e9a94160631ef596b0708b16be6c24f9a7978fea6fd09c4b9ebafefad71c2ad9  $ROOT/falling-dice-maintained-source.tar.xz" | sha256sum -c -
echo "Reconstructed: $ROOT/falling-dice-maintained-source.tar.xz"

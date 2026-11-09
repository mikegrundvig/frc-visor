#!/usr/bin/env bash
# Publishes this release of Visor into a clone of github.com/mikegrundvig/frc-maven: the robot client,
# and the coprocessor program the tag's release workflow built for arm64. Run after pushing the tag
# (vYEAR.MONTH.DAY, Cargo.toml's version) and once its GitHub release exists, from a clean checkout
# of that tag:
#
#   tools/release.sh [<frc-maven clone>]    ../frc-maven by default
#
# GRADLEW=<command> runs another wrapper around the client's ./gradlew. The clone's own commit and
# push put the release online.
set -euo pipefail
repo="$(cd "$(dirname "$0")/.." && pwd)"
maven="$(cd "${1:-$repo/../frc-maven}" && pwd)"
gradlew="${GRADLEW:-./gradlew}"
version="$(sed -n 's/^version = "\(.*\)"/\1/p' "$repo/Cargo.toml" | head -1)"
name="visor-$version-linux-arm64.tar.gz"
downloads="${XDG_CACHE_HOME:-$HOME/.cache}/visor-releases/v$version"

# The client is built from this working tree, and the program from the tag: they must be the same
# code, since frames carry only the version ID to tell them apart.
tag="$(git -C "$repo" describe --exact-match HEAD 2>/dev/null || true)"
if [ "$tag" != "v$version" ]; then
  echo "HEAD isn't the tag v$version (Cargo.toml's version): check out the release's tag first" >&2
  exit 1
fi
if [ -n "$(git -C "$repo" status --porcelain)" ]; then
  echo "the working tree has changes: the client would be published from them, not from v$version" >&2
  exit 1
fi

mkdir -p "$downloads"
gh release download "v$version" --repo mikegrundvig/frc-visor --dir "$downloads" --clobber \
  --pattern "$name" --pattern "$name.sha256"
(cd "$downloads" && sha256sum --check "$name.sha256")

(cd "$repo/client" && $gradlew publishToFrcMaven "-PfrcMavenDir=$maven" "-PvisorTarball=$downloads/$name")
echo "Published Visor $version into $maven. To put it online, commit and push there."

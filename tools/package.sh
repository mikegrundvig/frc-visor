#!/usr/bin/env bash
# Packs a built Visor program into its release archive, visor-<version>-linux-arm64.tar.gz
# (Cargo.toml's version), as the release workflow attaches it to the tag's GitHub release:
#
#   tools/package.sh <program> <out dir>      prints the archive's path
#
# The archive holds the program, its licence and notices (NAPA's, and a list of the Rust crates it's
# built from and their licences, which needs cargo-deny), the chrony configuration to start from,
# Visor's Spotter pack (spotter/), and os/: what a board's image installs at its root for Visor, its
# account, its folder, its unit, and what the agent needs to run Visor's pack (spotter/visor/README.md).
# The same files make the same archive: fixed owner, order and times, no gzip timestamp.
set -euo pipefail
# Folders 0755 and files as given, whatever the caller's umask.
umask 022
repo="$(cd "$(dirname "$0")/.." && pwd)"
program="$1"
out="$2"
version="$(sed -n 's/^version = "\(.*\)"/\1/p' "$repo/Cargo.toml" | head -1)"
name="visor-$version-linux-arm64.tar.gz"

mkdir -p "$out"
package="$(mktemp -d "$out/package.XXXXXX")"
chmod 0755 "$package"
trap 'rm -rf "$package"' EXIT

install -m 0755 "$program" "$package/visor"
install -m 0644 "$repo/LICENSE" "$repo/NOTICE" "$repo/deploy/chrony.conf" "$package/"
# NAPA's notice and its own third-party notices, where NOTICE says they are.
install -D -m 0644 "$repo/vendor/napa/NOTICE" "$package/vendor/napa/NOTICE"
install -m 0644 "$repo/vendor/napa/THIRD_PARTY_NOTICES" "$package/vendor/napa/"
# Each Rust crate the program is built from and its licences, from Cargo.lock by cargo-deny: the
# arm64 build's crates, less the tests' own.
{
  echo "The Rust crates Visor's program is built from, each with its licences, from Cargo.lock"
  echo "(cargo deny list). Each crate's licence text is in its source, on crates.io."
  echo
  cargo deny --log-level error --manifest-path "$repo/Cargo.toml" --locked --offline \
    --exclude-dev --target aarch64-unknown-linux-gnu --color never list --layout crate |
    grep -v '^visor@'
} >"$package/THIRD-PARTY-LICENSES"
chmod 0644 "$package/THIRD-PARTY-LICENSES"
# Visor's Spotter pack, which the robot program pushes to its boards.
mkdir "$package/spotter"
cp -r "$repo/spotter/visor" "$package/spotter/"
# The image's part, at the paths a package's own files take, so the board's /etc can still override
# any of them.
os="$package/os"
install -D -m 0644 "$repo/deploy/visor.service" "$os/usr/lib/systemd/system/visor.service"
install -D -m 0644 "$repo/deploy/visor.sysusers" "$os/usr/lib/sysusers.d/visor.conf"
install -D -m 0644 "$repo/deploy/visor.tmpfiles" "$os/usr/lib/tmpfiles.d/visor.conf"
install -D -m 0644 "$repo/deploy/frc-spotter-visor.conf" "$os/usr/lib/systemd/system/frc-spotter.service.d/visor.conf"
install -D -m 0644 "$repo/deploy/50-frc-spotter-visor.rules" "$os/usr/share/polkit-1/rules.d/50-frc-spotter-visor.rules"

tar -C "$package" --owner=0 --group=0 --numeric-owner --sort=name \
  --mtime="@$(git -C "$repo" log -1 --format=%ct)" -cf - . | gzip -n >"$out/$name"
echo "$out/$name"

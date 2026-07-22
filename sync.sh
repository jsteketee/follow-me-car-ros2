#!/usr/bin/env bash
# sync.sh — deploy the Mac working tree to the Pi over SSH via rsync.
# Source of truth is the Mac; the Pi is a run-on-hardware target. Syncs SOURCE
# only — build artifacts (build/ install/ log/) are built on each machine, and
# per-machine files (CLAUDE.md, .git) are never overwritten.

set -euo pipefail

HOST="${HOST:-followme-pi}"
DEST="${DEST:-follow-me-car-ros2/}"          # path on the Pi, relative to ~
SRC="$(cd "$(dirname "$0")" && pwd)/"        # this repo (trailing slash = contents)

# What NOT to push to the Pi: build outputs, VCS, per-machine config, host-only
# dev files, editor cruft. The dir-merge filter also applies every .gitignore in
# the tree (root + web/) so gitignored artifacts stay on the Mac. This is an
# OVERLAY sync (no --delete): the Pi keeps its own files — rosbag recordings,
# logs, tf frame dumps — untouched. Deleting a file on the Mac does NOT remove it
# on the Pi; do that by hand when you intend it.
EXCLUDES=(
  --filter=':- .gitignore'
  --exclude 'build/'          --exclude 'install/'      --exclude 'log/'
  --exclude '.git/'           --exclude '.DS_Store'     --exclude '.pytest_cache/'
  --exclude '__pycache__/'    --exclude '*.py[cod]'
  --exclude 'CLAUDE.md'       --exclude '.claude/'      --exclude '.vscode/'
  --exclude '.devcontainer/'  --exclude 'node_modules/' --exclude 'dist/'
)

# Dry-run by default; pass --apply to actually transfer.
MODE="--dry-run"
BANNER="DRY RUN (no changes) — pass --apply to sync for real"
if [ "${1:-}" = "--apply" ]; then
  MODE=""
  BANNER="APPLYING sync ${SRC} -> ${HOST}:~/${DEST}"
fi

echo "== $BANNER =="
rsync -avz $MODE "${EXCLUDES[@]}" "$SRC" "${HOST}:${DEST}"
[ -n "$MODE" ] && echo "== dry run complete. Re-run with:  ./sync.sh --apply ==" || echo "== sync complete =="

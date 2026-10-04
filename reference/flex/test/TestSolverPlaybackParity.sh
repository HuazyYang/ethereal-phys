#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
oracle="${repo_root}/bin/win64/Pot Pourri_playbackbuffer"
build_root="${NVFLEX_BUILD_ROOT:-${repo_root}/build/wine-clang}"
smoke_seconds="${NVFLEX_PLAYBACK_SMOKE_SECONDS:-10}"

if [[ ! -f "${oracle}" ]]; then
    echo "Missing playback oracle: ${oracle}" >&2
    echo "Generate it with --dev=0 --playback-mode=write --playback-range=0,200." >&2
    exit 2
fi

log_dir="$(mktemp -d)"
log_file="${log_dir}/playback-read.log"
cleanup() {
    rm -f "${log_file}"
    rmdir "${log_dir}"
}
trap cleanup EXIT

set +e
NVFLEX_BUILD_ROOT="${build_root}" \
NVFLEX_WINEDEBUG=-all \
"${repo_root}/scripts/wine/run-demo.sh" \
    --config Debug \
    --rhi d3d11 \
    --dev 1 \
    --smoke-seconds "${smoke_seconds}" \
    -- \
    --playback-mode=read >"${log_file}" 2>&1
status=$?
set -e

if ((status != 0)); then
    cat "${log_file}"
    echo "Reconstructed solver playback exited with status ${status}." >&2
    exit "${status}"
fi

if grep -q "particle buffer mis-coincident" "${log_file}"; then
    grep -m 2 "\[Playback\]\[Frame" "${log_file}" >&2
    echo "Reconstructed solver diverged from the vendor particle oracle." >&2
    exit 1
fi

echo "Reconstructed solver matches the vendor particle oracle."

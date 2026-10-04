#!/usr/bin/env bash

set -u

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_root="${NVFLEX_BUILD_ROOT:-/tmp/nvflex-wine-clang}"
frame_start="${NVFLEX_PLAYBACK_FRAME_START:-0}"
frame_end="${NVFLEX_PLAYBACK_FRAME_END:-200}"
smoke_seconds="${NVFLEX_PLAYBACK_SMOKE_SECONDS:-4}"
retry_seconds="${NVFLEX_PLAYBACK_RETRY_SECONDS:-12}"
log_root="${NVFLEX_PLAYBACK_LOG_ROOT:-/tmp/nvflex-all-scenes-parity}"
runner="$repo_root/scripts/wine/run-demo.sh"

scenes=(
    "Pot Pourri"
    "Soft Octopus"
    "Soft Teapot"
    "Soft Rope"
    "Soft Cloth"
    "Soft Bowl"
    "Soft Rod"
    "Soft Armadillo"
    "Soft Bunny"
    "Plastic Bunnies"
    "Plastic Comparison"
    "Plastic Stack"
    "Friction Ramp"
    "Friction Moving Box"
    "Friction Moving Sphere"
    "Friction Moving Capsule"
    "Friction Moving Mesh"
    "Shape Collision"
    "Shape Channels"
    "Triangle Collision"
    "Local Space Fluid"
    "Local Space Cloth"
    "World Space Fluid"
    "Env Cloth Small"
    "Env Cloth Large"
    "Flag Cloth"
    "Inflatables"
    "Cloth Layers"
    "Sphere Cloth"
    "Tearing"
    "Pasta"
    "Game Mesh Rigid"
    "Game Mesh Particles"
    "Game Mesh Fluid"
    "Game Mesh Cloth"
    "Rigid Debris"
    "Viscosity Low"
    "Viscosity Med"
    "Viscosity High"
    "Adhesion"
    "Goo Gun"
    "Buoyancy"
    "Melting"
    "Surface Tension Low"
    "Surface Tension Med"
    "Surface Tension High"
    "DamBreak  5cm"
    "DamBreak 10cm"
    "DamBreak 15cm"
    "Rock Pool"
    "Rayleigh Taylor 2D"
    "Trigger Volume"
    "Force Field"
    "Initial Overlap"
    "Rigid2"
    "Rigid4"
    "Rigid8"
    "Bananas"
    "Low Dimensional Shapes"
    "Granular Pile"
    "Parachuting Bunnies"
    "Water Balloons"
    "Rigid Fluid Coupling"
    "Fluid Block"
    "Fluid Cloth Coupling Water"
    "Fluid Cloth Coupling Goo"
    "Bunny Bath Dam"
)

mkdir -p "$log_root"
results="$log_root/results.tsv"
printf 'index\tscene\tstatus\tmismatch_frames\tdetail\n' >"$results"

run_demo()
{
    local dev="$1"
    local seconds="$2"
    local log="$3"
    shift 3

    DXVK_LOG_LEVEL=none NVFLEX_BUILD_ROOT="$build_root" \
        "$runner" --config Debug --rhi d3d11 --dev "$dev" \
        --smoke-seconds "$seconds" -- "$@" >"$log" 2>&1
}

has_completion()
{
    local log="$1"
    grep -Fq "[Playback] completed frame range [$frame_start,$frame_end)" "$log"
}

validate_oracle()
{
    local oracle="$1"
    python3 - "$oracle" "$frame_start" "$frame_end" <<'PY'
import struct
import sys

path = sys.argv[1]
expected_start = int(sys.argv[2])
expected_end = int(sys.argv[3])
with open(path, "rb") as stream:
    header = stream.read(8)
    if len(header) != 8:
        raise SystemExit("short header")
    start, end = struct.unpack("<II", header)
    if (start, end) != (expected_start, expected_end):
        raise SystemExit(f"range is [{start},{end})")
    for frame in range(start, end):
        count_data = stream.read(4)
        if len(count_data) != 4:
            raise SystemExit(f"missing particle count at frame {frame}")
        count = struct.unpack("<I", count_data)[0]
        positions = stream.read(count * 16)
        if len(positions) != count * 16:
            raise SystemExit(f"short position buffer at frame {frame}")
    if stream.read(1):
        raise SystemExit("trailing data")
PY
}

for index in "${!scenes[@]}"; do
    scene="${scenes[$index]}"
    write_log="$log_root/$(printf '%02d' "$index")-write.log"
    read_log="$log_root/$(printf '%02d' "$index")-read.log"
    oracle="$repo_root/bin/win64/${scene}_playbackbuffer"

    printf '[%02d/%02d] %-32s ' "$index" "$((${#scenes[@]} - 1))" "$scene"

    run_demo 0 "$smoke_seconds" "$write_log" \
        "--scene=$index" --playback-mode=write \
        "--playback-range=$frame_start,$frame_end"
    write_status=$?
    if ((write_status != 0)) || ! has_completion "$write_log"; then
        run_demo 0 "$retry_seconds" "$write_log" \
            "--scene=$index" --playback-mode=write \
            "--playback-range=$frame_start,$frame_end"
        write_status=$?
    fi

    if ((write_status != 0)) || ! has_completion "$write_log"; then
        printf 'WRITE_INCOMPLETE\n'
        printf '%d\t%s\tWRITE_INCOMPLETE\t0\t%s\n' \
            "$index" "$scene" "$write_log" >>"$results"
        continue
    fi

    oracle_error="$(validate_oracle "$oracle" 2>&1)"
    if [[ -n "$oracle_error" ]]; then
        printf 'ORACLE_INVALID\n'
        printf '%d\t%s\tORACLE_INVALID\t0\t%s\n' \
            "$index" "$scene" "$oracle_error" >>"$results"
        continue
    fi

    run_demo 1 "$smoke_seconds" "$read_log" \
        "--scene=$index" --playback-mode=read
    read_status=$?
    if ((read_status != 0)) || ! has_completion "$read_log"; then
        run_demo 1 "$retry_seconds" "$read_log" \
            "--scene=$index" --playback-mode=read
        read_status=$?
    fi

    if ((read_status != 0)) || ! has_completion "$read_log"; then
        printf 'READ_INCOMPLETE\n'
        printf '%d\t%s\tREAD_INCOMPLETE\t0\t%s\n' \
            "$index" "$scene" "$read_log" >>"$results"
        continue
    fi

    mismatch_count="$(grep -Fc 'particle buffer mis-coincident' "$read_log" || true)"
    if ((mismatch_count > 0)); then
        first_mismatch="$(grep -Fm1 '[Playback][Frame' "$read_log" || true)"
        printf 'FAIL (%s frames)\n' "$mismatch_count"
        printf '%d\t%s\tFAIL\t%s\t%s\n' \
            "$index" "$scene" "$mismatch_count" "$first_mismatch" >>"$results"
    else
        printf 'PASS\n'
        printf '%d\t%s\tPASS\t0\t\n' "$index" "$scene" >>"$results"
    fi
done

printf '\nResults: %s\n' "$results"
awk -F '\t' 'NR > 1 { count[$3]++ } END { for (status in count) print status, count[status] }' "$results" | sort

if awk -F '\t' 'NR > 1 && $3 != "PASS" { found=1 } END { exit !found }' "$results"; then
    exit 1
fi

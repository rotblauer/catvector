#!/usr/bin/env bash

# set -x
set -e

# CAT_ONE
CAT_ONE="${1:-rye}"

# TRACKS_SOURCE is the source of the tracks data. It can be a file, and should be a .json.gz.
TRACKS_SOURCE_GZ="${TRACKS_SOURCE_GZ:-$HOME/tdata/edge.json.gz}"

# OUTPUT_ROOT is the root directory where the generated data will be stored.
OUTPUT_ROOT="${OUTPUT_ROOT:-$HOME/tdata/local/catvector/gen}"
# If any output data already exists it will be nuked to avoid dirty or unreproducible output.
rm -rf "${OUTPUT_ROOT}"
mkdir -p "${OUTPUT_ROOT}"

# OUTPUT_REFERENCE is a copy of the refence file used to originate the data.
# Create a copy of the edge.json.gz file if it doesn't exist.
# It is important to keep and use a copy of the original source data
# to ensure that the pipeline's output is reproducible.
OUTPUT_REFERENCE="${OUTPUT_REFERENCE:-$OUTPUT_ROOT/reference/$(basename ${TRACKS_SOURCE_GZ})}"
mkdir -p "$(dirname ${OUTPUT_REFERENCE})"
[[ -f "${OUTPUT_REFERENCE}" ]] || cp "${TRACKS_SOURCE_GZ}" "${OUTPUT_REFERENCE}"

# Build the go program to avoid having to that repeatedly.
BUILD_TARGET=./build/bin/catvector
mkdir -p "$(dirname ${BUILD_TARGET})"
go build -o "${BUILD_TARGET}" ./main.go
export BUILD_TARGET

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

intermediary_gzipping() {
    # This function takes a directory path as its first argument and writes to temporary files in that directory.
    # Temporary files are useful to avoid gzip compression issues when running in parallel.
    # These temporary files can then be cat'ed together on completion.
    local outroot="${1}"
    local TMPDIR="${outroot}"
    mkdir -p "${outroot}"

#     mkdir -p "${1}"
#     local out_ndjson=""
#     local out_fc=""
    tee >(gzip > "$(mktemp -t tmp-XXXXXXXXXX.ndjson.gz)")
#     | tee >(ndgeojson2geojsonfc | gzip > "$(mktemp -t tmp-XXXXXXXXXX.fc.json.gz)")

#     | tee "${1}.ndjson" \
#     | tee >(ndgeojson2geojsonfc > "${1}.fc.json")
}
export -f intermediary_gzipping

process() {
    local ONECAT="${1}"
    local OUTPUT_ROOT_ONECAT="${2}"
    >&2 echo "Processing category: ${ONECAT}"
    cattracks-names modify \
    | gfilter --match-all '#(properties.Name=='"${ONECAT}"')' \
    | ${BUILD_TARGET} rkalman \
    | ${BUILD_TARGET} --interval=30s points-to-linestrings \
    | ${BUILD_TARGET} --threshold=0.00008 douglas-peucker \
    | tee >( \
        gfilter --match-all "#(properties.IsMoving==true),#(properties.Duration>30)" \
        | intermediary_gzipping "${OUTPUT_ROOT_ONECAT}/rkalman-linestrings-dp-moving" \
        | tee >(${SCRIPT_DIR}/runt.sh "${OUTPUT_ROOT_ONECAT}/laps.mbtiles" laps) \
        ) \
    | tee >( \
        gfilter --match-all "#(properties.IsMoving==false)" \
        | $BUILD_TARGET linestrings-to-points \
        | intermediary_gzipping "${OUTPUT_ROOT_ONECAT}/points-stationary" \
        | tee >(${SCRIPT_DIR}/runt.sh "${OUTPUT_ROOT_ONECAT}/naps.mbtiles" naps) \
        ) \


}
export -f process

percat() {
    local ONECAT="${1}"
    local OUTPUT_ROOT_ONECAT="${2}"
    parallel -j 6 --pipe -L 500000 process "${ONECAT}" "${OUTPUT_ROOT_ONECAT}" \
    > /dev/null

#     for d in "$(find ${OUTPUT_ROOT_ONECAT} -type d)"; do
#         # Concatenate the temporary files together.
#         cat "${d}"/*.ndjson.gz > "${d}.ndjson.gz"
#         cat "${d}"/*.fc.json.gz > "${d}.fc.json.gz"
#         # And remove the original temporary files.
#         rm -rf "${d}/tmp-*"
#     done
#
#     echo "----------------"
#     echo "CATEGORY: ${ONECAT}"
#     echo "MOVING:"
#     echo "Feature count: $(zcat ${OUTPUT_ROOT_ONECAT}/rkalman-linestrings-dp-moving.ndjson.gz | wc -l)"
#     echo -n "Seconds: "
#     zcat ${OUTPUT_ROOT_ONECAT}/rkalman-linestrings-dp-moving.ndjson.gz | while read -r line; do jj 'properties.Duration' <<< $line ; done | awk '{s+=$1} END {print s}'
#     echo "STATIONARY:"
#     echo "Feature count: $(zcat ${OUTPUT_ROOT_ONECAT}/rkalman-linestrings-dp-stationary.ndjson.gz | wc -l)"
#     echo -n "Seconds: "
#     zcat ${OUTPUT_ROOT_ONECAT}/rkalman-linestrings-dp-stationary.ndjson.gz | while read -r line; do jj 'properties.Duration' <<< $line ; done | awk '{s+=$1} END {print s}'
}

run() {
    OUTPUT_ROOT_CAT_ONE="${OUTPUT_ROOT}/${CAT_ONE}" # eg. <OUTPUT_ROOT>/rye
    mkdir -p "${OUTPUT_ROOT_CAT_ONE}"
    percat "${CAT_ONE}" "${OUTPUT_ROOT_CAT_ONE}"
}

run

# cattracks explorer
# http://localhost:8080/public/?geojson=http://localhost:8000/rye/reference.fc.json,http://localhost:8000/rye/rkalman-linestrings-dp-stationary.fc.json

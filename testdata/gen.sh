#!/usr/bin/env bash

# set -x
set -e

# TRACKS_SOURCE is the source of the tracks data. It can be a file, and should be a .json.gz.
TRACKS_SOURCE_GZ="${TRACKS_SOURCE_GZ:-$HOME/tdata/edge.json.gz}"

# OUTPUT_ROOT is the root directory where the generated data will be stored.
OUTPUT_ROOT="${OUTPUT_ROOT:-$HOME/tdata/local/gen}"
rm -rf "${OUTPUT_ROOT}"
mkdir -p "${OUTPUT_ROOT}"

# OUTPUT_REFERENCE is a copy of the refence file used to originate the data.
# Create a copy of the edge.json.gz file if it doesn't exist.
# It is important to keep and use a copy of the original source data
# to ensure that the pipeline's output is reproducible.
OUTPUT_REFERENCE="${OUTPUT_REFERENCE:-$OUTPUT_ROOT/reference/$(basename ${TRACKS_SOURCE_GZ})}"
mkdir -p "$(dirname ${OUTPUT_REFERENCE})"
[[ -f "${OUTPUT_REFERENCE}" ]] || cp "${TRACKS_SOURCE_GZ}" "${OUTPUT_REFERENCE}"

intermediary_gzipping() {
    tee >(gzip > "${1}.ndjson.gz") \
    | tee >(ndgeojson2geojsonfc | gzip > "${1}.fc.json.gz") \
    | tee "${1}.ndjson" \
    | tee >(ndgeojson2geojsonfc > "${1}.fc.json")
}

percat() {
    local ONECAT="${1}"
    local OUTPUT_ROOT_ONECAT="${2}"
    zcat "${OUTPUT_REFERENCE}" \
    | cattracks-names modify \
    | gfilter --match-all '#(properties.Name=='"${ONECAT}"')' \
    | intermediary_gzipping "${OUTPUT_ROOT_ONECAT}/reference" \
    | go run main.go rkalman \
    | intermediary_gzipping "${OUTPUT_ROOT_ONECAT}/rkalman" \
    | go run main.go --interval=30s points-to-linestrings \
    | intermediary_gzipping "${OUTPUT_ROOT_ONECAT}/rkalman-linestrings" \
    | go run main.go --threshold=0.00008 douglas-peucker \
    | intermediary_gzipping "${OUTPUT_ROOT_ONECAT}/rkalman-linestrings-dp" \
    | tee >( \
        gfilter --match-all "#(properties.IsMoving==true)" \
        | intermediary_gzipping "${OUTPUT_ROOT_ONECAT}/rkalman-linestrings-dp-moving") \
    | tee >( \
        gfilter --match-all "#(properties.IsMoving==false)" \
        | intermediary_gzipping "${OUTPUT_ROOT_ONECAT}/rkalman-linestrings-dp-stationary") \
    > /dev/null

    echo "----------------"
    echo "CATEGORY: ${ONECAT}"
    echo "MOVING:"
    echo "Feature count: $(zcat ${OUTPUT_ROOT_ONECAT}/rkalman-linestrings-dp-moving.ndjson.gz | wc -l)"
    echo -n "Seconds: "
    zcat ${OUTPUT_ROOT_ONECAT}/rkalman-linestrings-dp-moving.ndjson.gz | while read -r line; do jj 'properties.Duration' <<< $line ; done | awk '{s+=$1} END {print s}'
    echo "STATIONARY:"
    echo "Feature count: $(zcat ${OUTPUT_ROOT_ONECAT}/rkalman-linestrings-dp-stationary.ndjson.gz | wc -l)"
    echo -n "Seconds: "
    zcat ${OUTPUT_ROOT_ONECAT}/rkalman-linestrings-dp-stationary.ndjson.gz | while read -r line; do jj 'properties.Duration' <<< $line ; done | awk '{s+=$1} END {print s}'
}

cats=(
    "rye"
    "ia"
    )

for CAT_ONE in "${cats[@]}"; do
    echo "Processing category: ${CAT_ONE}"
    OUTPUT_ROOT_CAT_ONE="${OUTPUT_ROOT}/${CAT_ONE}" # eg. <OUTPUT_ROOT>/rye

    mkdir -p "${OUTPUT_ROOT_CAT_ONE}"
    percat "${CAT_ONE}" "${OUTPUT_ROOT_CAT_ONE}"
done

# cattracks explorer
# http://localhost:34785/public/?sources=http://localhost:8000/edge-rye-tail.fc.json,http://localhost:8000/edge-rye-tail-kalman.fc.json
# http://localhost:34785/public/?sources=http://localhost:8000/edge-rye-tail.fc.json,http://localhost:8000/edge-rye-tail-kalman-linestrings-dp.fc.json

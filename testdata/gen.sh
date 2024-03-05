#!/usr/bin/env bash

set -x
set -e

# The directory where this script is.
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# Create a copy of the edge.json.gz file if it doesn't exist.
[[ -f "${SCRIPT_DIR}/edge.json.gz" ]] || cp ~/tdata/edge.json.gz "${SCRIPT_DIR}/edge.json.gz"

# Create the rye edge feature collection.
zcat "${SCRIPT_DIR}/edge.json.gz" \
    | gfilter --match-all '#(properties.Alias=="rye")' \
    | tee "${SCRIPT_DIR}/edge-rye.json" \
    | ndgeojson2geojsonfc \
    > "${SCRIPT_DIR}/edge-rye.fc.json"

# Create a TAILED rye edge feature collection.
zcat "${SCRIPT_DIR}/edge.json.gz" \
    | gfilter --match-all '#(properties.Alias=="rye")' \
    | tail -10000 \
    | tee "${SCRIPT_DIR}/edge-rye-tail.json" \
    | ndgeojson2geojsonfc \
    > "${SCRIPT_DIR}/edge-rye-tail.fc.json"

cat "${SCRIPT_DIR}/edge-rye-tail.json" \
    | go run main.go rkalman \
    | tee "${SCRIPT_DIR}/edge-rye-tail-kalman.json" \
    | ndgeojson2geojsonfc \
    > "${SCRIPT_DIR}/edge-rye-tail-kalman.fc.json"

cat "${SCRIPT_DIR}/edge-rye-tail-kalman.json" \
  | go run main.go --interval=30s points-to-linestrings \
  | gfilter --match-all "#(properties.Duration>30)" \
  | gfilter --match-all "#(properties.IsMoving==true)" \
  | tee "${SCRIPT_DIR}/edge-rye-tail-kalman-linestrings.json" \
  | ndgeojson2geojsonfc > "${SCRIPT_DIR}/edge-rye-tail-kalman-linestrings.fc.json"

cat "${SCRIPT_DIR}/edge-rye-tail-kalman-linestrings.json" \
  | go run main.go --threshold=0.00008 douglas-peucker \
  | tee "${SCRIPT_DIR}/edge-rye-tail-kalman-linestrings-dp.json" \
  | ndgeojson2geojsonfc > "${SCRIPT_DIR}/edge-rye-tail-kalman-linestrings-dp.fc.json"

# cattracks explorer
# http://localhost:34785/public/?sources=http://localhost:8000/edge-rye-tail.fc.json,http://localhost:8000/edge-rye-tail-kalman.fc.json
# http://localhost:34785/public/?sources=http://localhost:8000/edge-rye-tail.fc.json,http://localhost:8000/edge-rye-tail-kalman-linestrings-dp.fc.json

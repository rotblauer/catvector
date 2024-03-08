#!/usr/bin/env bash

# set -x
set -e

export PARALLEL_BATCH_SIZE=${PARALLEL_BATCH_SIZE:-500000}

#######################################
# This function takes a directory path as its first argument and writes to temporary files in that directory.
# Temporary files are useful to avoid gzip compression issues when running in parallel.
# These temporary files can then be cat'ed together on completion.
# Arguments:
#   1  The directory path to write temporary files to.
#######################################
intermediary_gzipping_tmp() {
  local TMPDIR="${1}"
  mkdir -p "${TMPDIR}"

  tee >(gzip >"$(mktemp -t tmp-XXXXXXXXXX.ndjson.gz)")
  #     | tee >(ndgeojson2geojsonfc | gzip > "$(mktemp -t tmp-XXXXXXXXXX.fc.json.gz)")
  #     | tee "${1}.ndjson" \
  #     | tee >(ndgeojson2geojsonfc > "${1}.fc.json")
}
export -f intermediary_gzipping_tmp

#######################################
# Tees the input to a gzip file and the output.
# Arguments:
#   1 - the gzip file to which to write the input, gzipped
#######################################
intermediary_gzipping_to() {
  local out_file="${1}"
  mkdir -p "$(dirname "${out_file}")"

  tee >(gzip >"${out_file}")
  #     | tee >(ndgeojson2geojsonfc | gzip > "$(mktemp -t tmp-XXXXXXXXXX.fc.json.gz)")
  #     | tee "${1}.ndjson" \
  #     | tee >(ndgeojson2geojsonfc > "${1}.fc.json")
}
export -f intermediary_gzipping_to

#######################################
# process the parallelized data, per cat.
# Globals:
#   BUILD_TARGET
#   CAT_ONE
#   OUTPUT_ROOT_CAT_ONE
# Arguments:
#  None
#######################################
process() {
  #     | ${BUILD_TARGET} rkalman \
  local batch_id
  printf -v batch_id "%04d" "${1}"
  [[ -z "${CAT_ONE}" ]] && echo "CAT_ONE is not set" && exit 1
  [[ -z "${OUTPUT_ROOT_CAT_ONE}" ]] && echo "OUTPUT_ROOT_CAT_ONE is not set" && exit 1
  [[ -z "${BUILD_TARGET}" ]] && echo "BUILD_TARGET is not set" && exit 1

  local completed_file
  completed_file="${OUTPUT_ROOT_CAT_ONE}/completed/batch-${batch_id}.txt"
  if [[ -f "${completed_file}" ]]; then
    echo >&2 "Skipping batch ${batch_id}. File exists: ${completed_file}"
    return
  fi

  echo >&2 "Processing category: ${CAT_ONE}, batch: ${batch_id}"
  ${BUILD_TARGET} validate \
    | cattracks-names modify-json --modify.get='properties.Name' --modify.set='properties.Name' \
    | gfilter --match-all '#(properties.Name=='"${CAT_ONE}"')' \
    | intermediary_gzipping_to "${OUTPUT_ROOT_CAT_ONE}/valid/batch-${batch_id}.json.gz" \
    | ${BUILD_TARGET} --interval=30s points-to-linestrings \
    | ${BUILD_TARGET} --threshold=0.00008 douglas-peucker \
    | tee >(
      gfilter --match-all "#(properties.IsMoving==true),#(properties.Duration>30)" \
        | intermediary_gzipping_to "${OUTPUT_ROOT_CAT_ONE}/linestrings/batch-${batch_id}.json.gz"
    ) \
    | tee >(
      gfilter --match-all "#(properties.IsMoving==false)" \
        | $BUILD_TARGET linestrings-to-points \
        | intermediary_gzipping_to "${OUTPUT_ROOT_CAT_ONE}/points/batch-${batch_id}.json.gz"
    )

    mkdir -p "$(dirname "${completed_file}")" && date > "${completed_file}"
}
export -f process

#######################################
# description
# Arguments:
#  None
#######################################
onecat() {
  parallel -j 6 --pipe -L "${PARALLEL_BATCH_SIZE}" process {#} \
    >/dev/null
}

#######################################
# description
# Globals:
#   BASH_SOURCE
#   CAT_ONE
#   OUTPUT_REFERENCE
#   OUTPUT_ROOT_CAT_ONE
# Arguments:
#  None
#######################################
main() {
  local script_dir
  script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)
  source "${script_dir}/setup.sh"

  # Skip any existing output.
#  [[ -d "${OUTPUT_ROOT_CAT_ONE}" ]] && echo "OUTPUT_ROOT_CAT_ONE already exists: ${OUTPUT_ROOT_CAT_ONE}" && exit 0

  mkdir -p "${OUTPUT_ROOT_CAT_ONE}"
  zcat "${OUTPUT_REFERENCE}" | onecat
}
main

# cattracks explorer
# http://localhost:8080/public/?geojson=http://localhost:8000/rye/reference.fc.json,http://localhost:8000/rye/rkalman-linestrings-dp-stationary.fc.json
# http://localhost:8080/public/?vector=http://localhost:3001/services/rye/naps/tiles/{z}/{x}/{y}.pbf,http://localhost:3001/services/rye/laps/tiles/{z}/{x}/{y}.pbf

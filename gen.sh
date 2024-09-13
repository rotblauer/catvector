#!/usr/bin/env bash

# set -x
set -e

# BATCH SIZE 500000 is normal for master processing.
export PARALLEL_BATCH_SIZE=${PARALLEL_BATCH_SIZE:-500000}
export PARALLEL_JOBS=${PARALLEL_JOBS:-10}
# Using 100000 for testing/trials.
#export PARALLEL_BATCH_SIZE=${PARALLEL_BATCH_SIZE:-100000}

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
#  batch_id
#######################################
process() {
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

  #    | ${BUILD_TARGET} rkalman \
  echo >&2 "Processing category: ${CAT_ONE}, batch: ${batch_id}"
  ${BUILD_TARGET} validate \
    | cattracks-names modify-json --modify.get='properties.Name' --modify.set='properties.Name' \
    | gfilter --match-all '#(properties.Name=='"${CAT_ONE}"')' \
    | intermediary_gzipping_to "${OUTPUT_ROOT_CAT_ONE}/valid/batch-${batch_id}.json.gz" \
    | ${BUILD_TARGET} --urban-canyon-distance=200 preprocess \
    | ${BUILD_TARGET} --dwell-interval=120s --dwell-distance=15 --trip-start-interval=30s --speed-threshold=0.5 trip-detector \
    | tee >( \
      gfilter --ignore-invalid --match-all '#(properties.IsTrip==true)' \
        | ${BUILD_TARGET} --dwell-interval=60s points-to-linestrings \
        | ${BUILD_TARGET} --threshold=0.00008 douglas-peucker \
        | intermediary_gzipping_to "${OUTPUT_ROOT_CAT_ONE}/linestrings/batch-${batch_id}.json.gz" \
    ) \
    | tee >( \
      gfilter --ignore-invalid --match-all '#(properties.IsTrip==false),#(properties.MotionStateReason!="reset")' \
        | ${BUILD_TARGET} --dwell-distance=100 consolidate-stops \
        | intermediary_gzipping_to "${OUTPUT_ROOT_CAT_ONE}/points/batch-${batch_id}.json.gz" \
    )

  mkdir -p "$(dirname "${completed_file}")" && date >"${completed_file}"
}
export -f process

#######################################
# description
# Arguments:
#  None
#######################################
parallel_process() {
  parallel -j "${PARALLEL_JOBS}" --pipe -L "${PARALLEL_BATCH_SIZE}" process {#} \
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
  set -x
  set -e
  local script_dir
  script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)
  source "${script_dir}/setup.sh"

  # Print all important global variables.
  echo >&2 "CAT_ONE = ${CAT_ONE}"
  echo >&2 "BUILD_TARGET = ${BUILD_TARGET}"
  echo >&2 "TRACKS_SOURCE_GZ = ${TRACKS_SOURCE_GZ}"
  echo >&2 "OUTPUT_REFERENCE = ${OUTPUT_REFERENCE}"
  echo >&2 "OUTPUT_ROOT_CAT_ONE = ${OUTPUT_ROOT_CAT_ONE}"
  echo >&2 "PARALLEL_BATCH_SIZE = ${PARALLEL_BATCH_SIZE}"
  echo >&2 "PARALLEL_JOBS = ${PARALLEL_JOBS}"

  # Skip any existing output corresponding to the hash of the input (file) (OUTPUT_REFERENCE).
  local hash
  hash="$(sha1sum "${OUTPUT_REFERENCE}")"
  [[ -f "${OUTPUT_ROOT_CAT_ONE}/generated" ]] \
    && [[ "$(head -1 "${OUTPUT_ROOT_CAT_ONE}/generated")" == "${hash}" ]] \
    && echo "Generation for ${OUTPUT_REFERENCE} already done: ${OUTPUT_ROOT_CAT_ONE}. Exiting." && return

  mkdir -p "${OUTPUT_ROOT_CAT_ONE}"
  zcat "${OUTPUT_REFERENCE}" | parallel_process

  # We're done, store the hash of the input file to the 'generated' file.
  echo "${hash}" >"${OUTPUT_ROOT_CAT_ONE}/generated"
  date >>"${OUTPUT_ROOT_CAT_ONE}/generated"
}
main



document() {
cat <<EOF > /dev/null
# rotblauer/cattracks-explorer
http://localhost:8080/public/?geojson=http://localhost:8000/rye/reference.fc.json,http://localhost:8000/rye/rkalman-linestrings-dp-stationary.fc.json
http://localhost:8080/public/?vector=http://localhost:3001/services/rye/naps/tiles/{z}/{x}/{y}.pbf,http://localhost:3001/services/rye/laps/tiles/{z}/{x}/{y}.pbf

http://localhost:3001/services/ia/laps/tiles/{z}/{x}/{y}.pbf

mbtileserver --port 3001 --cors '*' -d /home/ia/tdata/local/catvector/batch-042x --verbose --enable-fs-watch

EOF

}

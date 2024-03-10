#!/usr/bin/env bash


# CAT_ONE
[[ -z "${CAT_ONE}" ]] && CAT_ONE="${1:-rye}"
[[ -z "${CAT_ONE}" ]] && echo "CAT_ONE is not set" && exit 1
export CAT_ONE

# TRACKS_SOURCE is the source of the tracks data. It can be a file, and should be a .json.gz.
export TRACKS_SOURCE_GZ="${TRACKS_SOURCE_GZ:-$HOME/tdata/edge.json.gz}"
[[ -z "${TRACKS_SOURCE_GZ}" ]] && echo "TRACKS_SOURCE_GZ is not set" && exit 1
[[ ! -f "${TRACKS_SOURCE_GZ}" ]] && echo "TRACKS_SOURCE_GZ does not exist ${TRACKS_SOURCE_GZ}" && exit 1

# OUTPUT_ROOT is the root directory where the generated data will be stored.
export OUTPUT_ROOT="${OUTPUT_ROOT:-$HOME/tdata/local/catvector/gen}"
[[ -z "${OUTPUT_ROOT}" ]] && echo "OUTPUT_ROOT is not set" && exit 1
mkdir -p "${OUTPUT_ROOT}"

export OUTPUT_ROOT_CAT_ONE="${2:-${OUTPUT_ROOT}/${CAT_ONE}}"
[[ -z "${OUTPUT_ROOT_CAT_ONE}" ]] && echo "OUTPUT_ROOT is not set" && exit 1

# OUTPUT_REFERENCE is a copy of the reference file used to originate the data.
# We create a copy of the reference file if it doesn't exist.
# It is important to keep and use a copy of the original source data
# to ensure that the pipeline's output is reproducible, and to stabilize
# the source against modification while the script runs.
export OUTPUT_REFERENCE="${OUTPUT_REFERENCE:-${OUTPUT_ROOT_CAT_ONE}/reference.json.gz}"
[[ -z "${OUTPUT_REFERENCE}" ]] && echo "OUTPUT_REFERENCE is not set" && exit 1
mkdir -p "$(dirname "${OUTPUT_REFERENCE}")"
[[ "${OUTPUT_REFERENCE}" != "${TRACKS_SOURCE_GZ}" ]] && \
  { cp "${TRACKS_SOURCE_GZ}" "${OUTPUT_REFERENCE}" || echo "failed to copy reference file" && exit 1; }

# Build the go program to avoid having to do that repeatedly.
export BUILD_TARGET=./build/bin/catvector
mkdir -p "$(dirname ${BUILD_TARGET})"
go build -o "${BUILD_TARGET}" ./main.go


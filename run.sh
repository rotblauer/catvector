#!/usr/bin/env bash

usage() {
  cat <<EOF
  time env PARALLEL_JOBS=8 PARALLEL_BATCH_SIZE=100000 ./run.sh |& tee run.out
EOF
}

document() {
  cat <<EOF
  This script is a shortcut to generate the catvector tiles for a specific batch_id.
  Those batches are pre-generated by gen.sh.
  The purpose of this is to QUICKLY experiment with different catvector programs and configurations,
  by reducing the input to the program to a chapter, instead of the whole book.

  http://localhost:8080/public/?vector=http://localhost:3001/services/rye/naps/tiles/{z}/{x}/{y}.pbf,http://localhost:3001/services/rye/laps/tiles/{z}/{x}/{y}.pbf,http://localhost:3001/services/rye/valid/tiles/{z}/{x}/{y}.pbf

  20240915
  http://localhost:8080/public/?vector=http://localhost:3001/services/ia/laps/tiles/{z}/{x}/{y}.pbf,http://localhost:3001/services/ia/naps/tiles/{z}/{x}/{y}.pbf,http://localhost:3001/services/ia/valid/tiles/{z}/{x}/{y}.pbf
EOF
}

main() {
  set -e

  # - batch-013* stl, tampa, sanfran, minneap, duluth
  # - batch-023* stl, w-s, boone, denver
  # - batch-033* was for me missoula, glacier, to grandma's to buying the Ranger, ryne standard lake dodger
  # - batch-042* is a nice testdata batch, it includes RAGBRAI 2023.
  # ... Later. I assume these are batches from master. (Or direct-master.)
  #     Shoulda nota.
  #     Shoulda also nota the batch size.
  local batch_id="batch-044"
  local kitty
  for kitty in ia rye; do
    export CAT_ONE="${kitty}"

    # FORK ALERT: Below are two forking chunks of code.
    # They are different ways of establishing a reference data set.
    # The first chunk is an evolution of tinkering with different batches
    # derived, I assume, from master.json.gz. IIRC it evolved to look
    # at subsets defined in batches from a pre-processed batching run
    # on master.json.gz.

    # --------------------------------------------------
    # FORK 1/2: This chunk concatenates the batch files into a single reference file.
    # --------------------------------------------------
#    export OUTPUT_ROOT="$HOME/tdata/local/catvector/experiment1"
    export OUTPUT_REFERENCE="${OUTPUT_ROOT}/${CAT_ONE}/reference.json.gz"
#    export TRACKS_SOURCE_GZ="${OUTPUT_REFERENCE}"
#
#    # Derive the trial file from the pre-generated batch_id-wildcard batch files.
#    # Concat gz files.
#    export CONCATS="${HOME}/tdata/local/catvector/catvector-20241008/direct-master/ia/valid.json.gz"
##
#    set -x
#    mkdir -p "$(dirname "${OUTPUT_REFERENCE}")"
#    >"${OUTPUT_REFERENCE}"
#    for f in "${CONCATS}"*; do
#      cat "${f}" >>"${OUTPUT_REFERENCE}"
#    done
#    { set +x; } 2>/dev/null

    # --------------------------------------------------
    # FORK 2/2: This chunk copies <edge.json.gz> as the simple original track data source.
    # --------------------------------------------------
    # OUTPUT_ROOT is the root directory for the data output of this run.
    # Change this to match the source data for this run.
     export OUTPUT_ROOT="$HOME/tdata/local/catvector/edge"
#    export OUTPUT_ROOT="$HOME/tdata/local/catvector/direct-master"
#    export OUTPUT_ROOT="$HOME/tdata/local/catvector/20241008"
    # OUTPUT_REFERENCE is a copy of the source data for some run.
    # Probably don't change this.
    export OUTPUT_REFERENCE="${OUTPUT_ROOT}/${CAT_ONE}/reference.json.gz"
    # TRACKS_SOURCE_GZ is the "original source" for the tracks pipeline; it will be a COPY of the source data defined above.
    # Probably don't change this.
    export TRACKS_SOURCE_GZ="${OUTPUT_REFERENCE}"

    # Hardcode copy the original source data into our run's version of it.
    # MODIFY THIS TO CHANGE THE SOURCE DATA FOR THIS RUN.
    set -x
    mkdir -p "$(dirname "${OUTPUT_REFERENCE}")"
    cp "${HOME}/tdata/edge.json.gz" "${OUTPUT_REFERENCE}"
#    cp "${HOME}/tdata/direct-master.json.gz" "${OUTPUT_REFERENCE}"
#    cp "${HOME}/tdata/local/catvector-20241008/direct-master/rye/valid.json.gz" "${OUTPUT_REFERENCE}"
    { set +x; } 2>/dev/null

    ## Run the scripts.
    local script_dir
    script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)

    set -x
    echo "Running ${script_dir}/gen.sh"
    time "${script_dir}/gen.sh"

    echo "Running ${script_dir}/tile.sh"
    time "${script_dir}/tile.sh"
  done
}
main

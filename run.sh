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
    # The way I handle idempotency is sporadic and undocumented
    # but surely seemed convenient and probably necessary at the time.

    # This chunk concatenates the batch files into a single reference file.
    # --------------------------------------------------
#    export OUTPUT_ROOT="$HOME/tdata/local/catvector/${batch_id}x"
#    export OUTPUT_REFERENCE="${OUTPUT_ROOT}/${CAT_ONE}/reference.json.gz"
#    export TRACKS_SOURCE_GZ="${OUTPUT_REFERENCE}"
#
#    # Derive the trial file from the pre-generated batch_id-wildcard batch files.
#    # Concat gz files.
#    export CONCATS="${HOME}/tdata/local/catvector/gen/${CAT_ONE}/valid/${batch_id}"
#
#    set -x
#    mkdir -p "$(dirname "${OUTPUT_REFERENCE}")"
#    >"${OUTPUT_REFERENCE}"
#    for f in "${CONCATS}"*; do
#      cat "${f}" >>"${OUTPUT_REFERENCE}"
#    done
#    { set +x; } 2>/dev/null

    # This chunk copies <edge.json.gz>.
    # It
    # --------------------------------------------------
    export OUTPUT_ROOT="$HOME/tdata/local/catvector/master"
    # OUTPUT_REFERENCE is a copy of the source data for some run.
    # For data integrity and reproducibility.
    export OUTPUT_REFERENCE="${OUTPUT_ROOT}/${CAT_ONE}/reference.json.gz"
    export TRACKS_SOURCE_GZ="${OUTPUT_REFERENCE}"
    set -x
    mkdir -p "$(dirname "${OUTPUT_REFERENCE}")"

    # Hardcode copy the original source data into our run's version of it.
    cp "${HOME}/tdata/master.json.gz" "${OUTPUT_REFERENCE}"
    { set +x; } 2>/dev/null


    ## Run the scripts.

    local script_dir
    script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)

    set -x
#    echo "Running ${script_dir}/gen.sh"
#    time "${script_dir}/gen.sh"

    echo "Running ${script_dir}/tile.sh"
    time "${script_dir}/tile.sh"
  done
}
main

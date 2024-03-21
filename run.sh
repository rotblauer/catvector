#!/usr/bin/env bash


main() {
  set -e

  # - batch-023* stl, w-s, boone, denver
  # - batch-033* was for me missoula, glacier, to grandma's to buying the Ranger, ryne standard lake dodger
  # - batch-042* is a nice testdata batch, it includes RAGBRAI 2023.
  local batch_id="batch-042"
  local kitty
  for kitty in ia; do
    export CAT_ONE="${kitty}"
    export OUTPUT_ROOT="$HOME/tdata/local/catvector/${batch_id}x"
    export OUTPUT_REFERENCE="${OUTPUT_ROOT}/${CAT_ONE}/reference.json.gz"
    export TRACKS_SOURCE_GZ="${OUTPUT_REFERENCE}"

    # Derive the trial file from the pre-generated batch_id-wildcard batch files.
    # Concat gz files.
    local concatables="${HOME}/tdata/local/catvector/gen/${CAT_ONE}/valid/${batch_id}"

    set -x
    mkdir -p "$(dirname "${OUTPUT_REFERENCE}")"
    >"${OUTPUT_REFERENCE}"
    for f in "${concatables}"*; do
      cat "${f}" >>"${OUTPUT_REFERENCE}"
    done
#    cat "${concatables}"* >>"${OUTPUT_REFERENCE}"
    { set +x; } 2>/dev/null

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

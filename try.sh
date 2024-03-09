#!/usr/bin/env bash

main() {
  set -e

  CAT_ONE=ia

  local trial_file
  trial_file="${HOME}/tdata/local/catvector/source/${CAT_ONE}/valid/trial.json.gz"
  local trial_source

  # batch-042* is a nice testdata batch, it includes RAGBRAI 2023.
  trial_source="${HOME}/tdata/local/catvector/gen/${CAT_ONE}/valid/batch-042"

  export CAT_ONE
  export OUTPUT_ROOT="$HOME/tdata/local/catvector/trial"
  export TRACKS_SOURCE_GZ="${trial_file}"

  mkdir -p "$(dirname "${trial_file}")"
  > "${trial_file}"
  cat "${trial_source}"* >> "${trial_file}"

  local script_dir
  script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)

  set -x
  echo "Running ${script_dir}/gen.sh"
  time "${script_dir}/gen.sh"

  echo "Running ${script_dir}/tile.sh"
  time "${script_dir}/tile.sh"

#  source "${script_dir}/setup.sh"
#  zcat "${OUTPUT_REFERENCE}" | ${script_dir}/runtpp.sh "${OUTPUT_ROOT_CAT_ONE}/original.mbtiles"

}
main

#!/usr/bin/env bash

#######################################
# concatenate all lines and points and generate tilesets from them.
# Globals:
#   BASH_SOURCE
#   CAT_ONE
#   OUTPUT_ROOT_CAT_ONE
#   script_dir
# Arguments:
#  None
#######################################
postprocess() {

  set -e
  local script_dir
  script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)
  source "${script_dir}/setup.sh"

  [[ -z "${CAT_ONE}" ]] && echo "CAT_ONE is not set" && exit 1
  [[ -z "${OUTPUT_ROOT_CAT_ONE}" ]] && echo "OUTPUT_ROOT_CAT_ONE is not set" && exit 1
  [[ ! -d "${OUTPUT_ROOT_CAT_ONE}" ]] && echo "No data for category: ${OUTPUT_ROOT_CAT_ONE}" && exit 1
  echo >&2 "Postprocessing category: ${CAT_ONE}"

  # Concat all the files into one, for both linestrings and points.
  cat "${OUTPUT_ROOT_CAT_ONE}"/linestrings/*.json.gz \
    | tee -a "${OUTPUT_ROOT_CAT_ONE}/linestrings.json.gz" \
    | zcat \
    | gfilter --match-all '#(properties.PointCount>30),#(properties.Duration<86000),#(properties.Duration>120),#(properties.AverageAccuracy<25)' \
    | gfilter --match-any '#(properties.Activity!="Stationary"),#(properties.DistanceAbsolute>100),#(properties.DistanceTraversed>250)' \
    | ${script_dir}/runtpl.sh "${OUTPUT_ROOT_CAT_ONE}/laps.mbtiles" || true

  # This can fail because I'm not handling points right now
  # in the pipeline.
  set +e

  [[ -d "${OUTPUT_ROOT_CAT_ONE}"/points ]] \
  && cat "${OUTPUT_ROOT_CAT_ONE}"/points/*.json.gz \
    | tee -a "${OUTPUT_ROOT_CAT_ONE}/points.json.gz" \
    | zcat \
    | gfilter --match-all '#(properties.Accuracy<50)' \
    | ${script_dir}/runtpp.sh "${OUTPUT_ROOT_CAT_ONE}/naps.mbtiles"

  # Map the VALID points. These are all points.
  # They are useful for comparing the laps and naps against the "raw" data.
  [[ -d "${OUTPUT_ROOT_CAT_ONE}"/tripdetected ]] \
  && cat "${OUTPUT_ROOT_CAT_ONE}"/tripdetected/*.json.gz \
    | tee -a "${OUTPUT_ROOT_CAT_ONE}/tripdetected.json.gz" \
    | zcat | ${script_dir}/runtpp.sh "${OUTPUT_ROOT_CAT_ONE}/tripdetected.mbtiles"

#  # Map the VALID points. These are all points.
#  # They are useful for comparing the laps and naps against the "raw" data.
  [[ -d "${OUTPUT_ROOT_CAT_ONE}"/valid ]] \
  && cat "${OUTPUT_ROOT_CAT_ONE}"/valid/*.json.gz \
    | tee -a "${OUTPUT_ROOT_CAT_ONE}/valid.json.gz" \
    | zcat | ${script_dir}/runtpp.sh "${OUTPUT_ROOT_CAT_ONE}/valid.mbtiles"

  # Remove the intermediate files.
  #     rm -rf "${OUTPUT_ROOT_CAT_ONE}"/linestrings
  #     rm -rf "${OUTPUT_ROOT_CAT_ONE}"/points
}

postprocess

doc(){
  cat <<EOF
                addLayerObject.type = 'line';
                addLayerObject.paint = paintFor('line');
                addLayerObject.filter = [
                    'all',
                    ['>', 'PointCount', 30],
                    ['<', 'Duration', 86000],
                    ['>', 'Duration', 120],
                    ['<', 'AverageAccuracy', 25],
                    [
                        'any',
                        ['!=', 'Activity', 'Stationary'],
                        ['>=', 'DistanceAbsolute', 100],
                        ['>=', 'DistanceTraversed', 250],
                    ],

                    // ['==', 'Activity', '
EOF
}
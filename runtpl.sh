#!/usr/bin/env bash

# TIPPECANOE for LINESTRINGS (or any other geometry type != Point)

set -e
set -x

OUTPUT_FILE=${1:-$HOME/tdata/local/generated.mbtiles}
TILESET_NAME=${2:-$(basename $OUTPUT_FILE .mbtiles)}
LAYER_NAME=${3:-$(basename $OUTPUT_FILE .mbtiles)}

tippeargs=(
    --maximum-tile-bytes 750000 # default 500000
#    --drop-densest-as-needed
    --drop-smallest-as-needed
    --minimum-zoom 3
    --maximum-zoom 18 # 1me

	# Keep these fields
    --include Name
    --include UUID
    --include Time
    --include StartTime
    --include UnixTime

    --include Activity
    --include PointCount
    --include Duration
    --include AverageAccuracy
    --include DistanceTraversed
    --include DistanceAbsolute
    --include AverageReportedSpeed
    --include AverageCalculatedSpeed
    --include ElevationGain
    --include ElevationLoss
    --include MotionStateReasonStart
    --include MotionStateReasonEnd

    --order-by UnixTime

    --single-precision
    --generate-ids

    --read-parallel
    -l ${LAYER_NAME}
    -n ${TILESET_NAME}
    --temporary-directory "/tmp"
    -o ${OUTPUT_FILE}
    --force
)

main(){
    mkdir -p $(dirname $OUTPUT_FILE)
    tippecanoe "${tippeargs[@]}"
}

main

# zcat ${in} | tippecanoe \
# 		--maximum-tile-bytes 350000 \
# 		--cluster-densest-as-needed \
# 		--cluster-distance=1 \
# 		--calculate-feature-density \
# 		-r1 \
# 		--minimum-zoom 3 \
# 		--maximum-zoom 18 \
# 		--include UnixTime \
# 		--include Activity \
# 		--include Elevation \
# 		--include Speed \
# 		--include Accuracy \
# 		-EUnixTime:max \
# 		-EElevation:max \
# 		-ESpeed:max \
# 		-EAccuracy:mean \
# 		--single-precision \
# 		-l ${tilesetname} \
# 		-n ${tilesetname} \
# 		-o ${out} \
# 		--force

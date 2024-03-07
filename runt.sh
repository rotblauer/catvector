#!/usr/bin/env bash

#!/usr/bin/env bash

set -e
set -x

out=${1:-$HOME/tdata/master_testing.mbtiles}
tilesetname=${2:-cattracks}

tippeargs=(
    --maximum-tile-bytes 500000 # default 500000
		--calculate-feature-density
# 		-r1 # drop rate
        --drop-smallest-as-needed
        --drop-densest-as-needed
# 		--increase-gamma-as-needed
# 		--coalesce-densest-as-needed
# 		--simplify-only-low-zooms
# 		--no-tile-size-limit
		--minimum-zoom 3
		--maximum-zoom 13 # 1me
		# these are the only columns we need for cat maps
    --include Name
    --include Time
    --include Duration
    --include StartTime
		--include UnixTime
		--include Activity
		# --include Elevation
		# --include Speed
		# --include Accuracy
		# -EUnixTime:max
		# -EElevation:max
		# -ESpeed:max
		# -EAccuracy:mean
		--read-parallel
		--single-precision # reduce doubles to singles, smaller tiles
		-l ${tilesetname}
		-n ${tilesetname}
		--temporary-directory "/tmp"
		-o ${out}
		--force
)

main(){
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

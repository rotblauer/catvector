#!/usr/bin/env bash

set -e
export TRACKS_SOURCE_GZ="/home/ia/tdata/master.json.gz"
kitties=(rye)
for kitty in "${kitties[@]}"
do 
  export CAT_ONE="${kitty}"
  echo "Running CAT_ONE=${CAT_ONE}"
  time ./gen.sh
#  time ./tile.sh
done

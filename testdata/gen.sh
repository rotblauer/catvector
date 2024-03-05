#!/usr/bin/env bash

# The directory where this script is.
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# zcat ~/tdata/edge.json.gz | catnames-cli modify | gfilter --match-all '#(properties.Name=="rye")'
zcat ~/tdata/edge.json.gz | gfilter --match-all '#(properties.Alias=="rye")' > "${SCRIPT_DIR}/edge-rye.json"

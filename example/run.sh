#!/bin/bash

SRC_DIR=$(dirname -- "${BASH_SOURCE[0]}")

if [ $# -gt 0 ]; then
	ALG="$1"
else
	ALG="jpsP"
fi

if [ $# -gt 1 ]; then
	SCEN="$2"
else
	SCEN="$SRC_DIR/arena2.map.scen"
fi

# command

"$SRC_DIR/build/warthog-jps" --alg "$ALG" --scen "$SCEN" --checkopt

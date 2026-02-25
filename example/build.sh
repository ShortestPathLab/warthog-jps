#!/bin/bash

SRC_DIR=$(dirname -- "${BASH_SOURCE[0]}")
cmake -B "$SRC_DIR/build" -S "$SRC_DIR/.."
cmake --build "$SRC_DIR/build"

#!/bin/sh
# Positive control for the determinism workflow: re-enables FMA contraction by appending -ffp-contract=fast after
# the project's -ffp-contract=off (the last one wins). Used as CMAKE_C_COMPILER_LAUNCHER="sh;<this file>".
exec "$@" -ffp-contract=fast

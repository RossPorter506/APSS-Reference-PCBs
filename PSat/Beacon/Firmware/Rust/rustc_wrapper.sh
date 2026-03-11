#!/bin/sh

# Runs rustc with the msp430-gcc tooling added to PATH
PATH="./msp430-gcc/bin:$PATH" exec "$@"

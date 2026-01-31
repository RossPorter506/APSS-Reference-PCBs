#!/bin/sh

# Stop when a command fails
set -e

MSPDEBUG_DRIVER=rf2500
# For the launchpad kit only
#MSPDEBUG_DRIVER=tilib

echo "Flashing..."
if [ -e ./uniflash/dslite.sh ]; then
    ./uniflash/dslite.sh --config=./uniflash/user_files/configs/MSP430FR2355.ccxml -u "$1"
else
    # Alternate flashing command
    ./mspdebug/bin/mspdebug $MSPDEBUG_DRIVER "prog \"$1\"" "verify \"$1\""
fi

if [ "$2" == "gdb" ]; then
    echo "Starting debug daemon"
    ./mspdebug/bin/mspdebug $MSPDEBUG_DRIVER gdb &

    echo "Starting GDB"
    ./msp430-gcc/bin/msp430-elf-gdb -x mspdebug.gdb "$1"

    # Kill background debugger
    pkill -P $$
elif [ "$2" == "debug" ]; then
    echo "Starting debug daemon headless"
    ./mspdebug/bin/mspdebug $MSPDEBUG_DRIVER gdb
fi

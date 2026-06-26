#!/bin/bash
# usage: ./run_compiler.sh <vehicle> <board>
# example: ./run_compiler.sh copter TH-7

VEHICLE=$1
BOARD=$2

if [ -z "$VEHICLE" ] || [ -z "$BOARD" ]; then
    echo "Usage: $0 <vehicle> <board>"
    echo "Example: $0 copter TH-7"
    exit 1
fi

CURRENT_DATETIME=$(date +'%Y%m%d_%H%M%S')
echo "DateTime: $CURRENT_DATETIME"
echo "Vehicle:  $VEHICLE"
echo "Board:    $BOARD"

mkdir -p firmware
rm -fr firmware/*

echo "~~~~~~~~~~~~~~~ start ~~~~~~~~~~~~~~~"
./waf configure --board "$BOARD"
./waf "$VEHICLE"

cp "build/$BOARD/bin/ardu$VEHICLE.apj" "firmware/${CURRENT_DATETIME}_${BOARD}_ardu${VEHICLE}.apj"

echo "~~~~~~~~~~~~~~~ done ~~~~~~~~~~~~~~~"
echo "Output: firmware/${CURRENT_DATETIME}_${BOARD}_ardu${VEHICLE}.apj"

#!/bin/bash

# Reads DTCs from all ECUs I have access to except for the airbag
# controller in Golf Mk4.
#
# All of them use KWP1281 except for the ESP Mk60 brakes controller,
# which uses KWP2000.
#
# CLI args are passed on to rustbucket, so launch with -v if you want
# verbose output.
#
# https://www.ross-tech.com/vag-com/cars/vwmkIV.html

bold=$(tput bold)
normal=$(tput sgr0)

declare -A ecus
ecus[0x01]="Engine"
ecus[0x02]="Transmission"
ecus[0x03]="Brakes (ASB/ESP)"
ecus[0x08]="HVAC"
ecus[0x17]="Instrument Cluster"
ecus[0x19]="CAN Gateway"
ecus[0x46]="Central Convenience"
ecus[0x56]="Radio"
ecus[0x76]="Parking Aid"

# Not tested:
# - airbag controller (0x15)
# - steering wheel (0x16)
# - all wheel drive (0x22)
# - central locking (0x35)
# - navigation (0x37)
# - xenon headlights (0x55)

sorted=( $(echo ${!ecus[@]} | tr ' ' $'\n' | sort) )
for id in "${sorted[@]}"; do
    if [ $id != 0x01 ]; then
        echo ""
        sleep 1
    fi

    echo "${bold}# ECU $id: ${ecus[$id]}${normal}"

    if [ $id == 0x03 ]; then
        protocol="kwp2000"
    else
        protocol="kwp1281"
    fi

    for attempt in {1..3}; do
        if [ $attempt != 1 ]; then
            echo "Failed, retrying after wait."
            sleep 3
        fi

        rustbucket $protocol --ecu $id read-dtcs $@
        if [ $? == 0 ]; then
            break
        fi
    done
done

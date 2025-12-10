#! /bin/bash

killall -9 vrserver     
killall -9 vrcompositor
killall -9 vrmonitor
killall -9 steam

sleep 2

steam steam://run/250820 & 

#! /bin/bash

killall -9 vrserver     
killall -9 vrcompositor
killall -9 vrmonitor
killall -9 vrwebhelper
killall -9 steam
killall -9 steamtours

sleep 5

steam steam://run/250820 2>&1 > steanvr_log.txt &
echo "SteamVR restarted. Log output in steanvr_log.txt"
exit 0

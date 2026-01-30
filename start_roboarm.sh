#!/bin/bash

source /home/pi/ROBOARM/roboarm/bin/activate
ngrok http 8080 --url https://flying-scorpion-neat.ngrok-free.app &
python3 /home/pi/ROBOARM/ManualPhase1/WebBasedIKV2/website_dev/app.py

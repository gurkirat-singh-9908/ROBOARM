#!/bin/bash

source ~/ROBOARM/roboenv/bin/activate
ngrok http 8080 --url https://flying-scorpion-neat.ngrok-free.app &
python3 ~/ROBOARM/ManualPhase1/WebBasedIKV2/website_dev/app.py

#!/bin/bash

if [ ! -d ~/ROBOARM/roboenv ]; then
    echo "roboenv not found, recreating..."
    python3 -m venv ~/ROBOARM/roboenv
    ~/ROBOARM/roboenv/bin/pip install -r ~/ROBOARM/ManualPhase1/WebBasedIKV2/website_dev/requirements.txt
fi

source ~/ROBOARM/roboenv/bin/activate
ngrok http 8080 --url https://flying-scorpion-neat.ngrok-free.app &
python3 ~/ROBOARM/ManualPhase1/WebBasedIKV2/website_dev/app.py

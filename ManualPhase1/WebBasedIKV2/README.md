# WebBasedIKV2 – Web-Based IK Control (Version 2)

Flask web application that provides manual and automatic control of the robotic arm via a browser.

## Structure

```
WebBasedIKV2/
├── controller.py        # Serial controller (sends commands to Arduino)
├── IKFinal.py           # Inverse kinematics solver
├── main.py / main2.py   # Entry points
└── website_dev/
    ├── app.py           # Flask application (routes, SocketIO)
    ├── param.py         # Arm parameters (link lengths, offsets)
    ├── requirements.txt # Python dependencies
    ├── templates/       # Jinja2 HTML templates
    │   ├── index.html              # Home page
    │   ├── MANUAL_CONTROL.html     # Slider-based manual control
    │   ├── AUTOMATIC_CONTROL.html  # Target-coordinate automatic control
    │   ├── Connect.html            # Serial port connection page
    │   └── errors/                 # 404 / 500 error pages
    ├── static/          # CSS, fonts, 3-D OBJ model of arm
    └── data_fetcher/
        ├── ik.py           # IK calculations for the web backend
        ├── fetch_data.py   # Serial data fetcher
        ├── main.py         # Data fetcher entry point
        ├── param.py        # Shared parameters
        └── Arduino/        # Arduino firmware for this version
            ├── Arduino.ino         # Main Arduino sketch
            └── updateServos.cpp/h  # Servo update helpers
```

## Running

```bash
cd website_dev
pip install -r requirements.txt
python app.py
```

Then open `http://<raspberry-pi-ip>:5000` in a browser.

## Dependencies
- Flask, Flask-SocketIO
- pyserial
- numpy

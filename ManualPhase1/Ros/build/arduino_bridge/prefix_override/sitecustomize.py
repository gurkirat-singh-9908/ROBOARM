import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/kirat/Desktop/ROBOARM/ManualPhase1/Ros/install/arduino_bridge'

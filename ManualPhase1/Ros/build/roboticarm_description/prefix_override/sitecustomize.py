import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/kv3/ROBOARM/ManualPhase1/Ros/install/roboticarm_description'

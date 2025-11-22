import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/arz-1017/task/task_ws/install/tortoisebot_gazebo'

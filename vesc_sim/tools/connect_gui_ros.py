import ctypes,importlib
from pathlib import Path
import live_gui,ros_bridge
source=Path(ros_bridge.__file__)
exec(compile(source.read_text(),str(source),'exec'),ros_bridge.__dict__)
c=live_gui.controller
if c.bridge:c.bridge.close()
prefix=Path('/home/shin/damvi_vesc_ros_ws/install/vesc_msgs/lib')
ctypes.CDLL(str(prefix/'libvesc_msgs__rosidl_generator_py.so'),mode=ctypes.RTLD_GLOBAL)
c.bridge=ros_bridge.RosBridge(c.config)
c.write_status()
print('ROS connected in existing GUI')

import json
import live_gui
_native_snapshot_c = live_gui.controller
_native_snapshot_n = _native_snapshot_c.native
_native_snapshot_o = _native_snapshot_n.output
print(json.dumps({
    'elapsed':_native_snapshot_c.elapsed,
    'drive_state':{key:getattr(_native_snapshot_n.drive,key) for key in ('current','center_twist','regen_joules')},
    'drive_output':{key:getattr(_native_snapshot_o,key) for key in ('current','shaft_omega','center_speed_error','center_torque','electrical_brake','erpm')},
    'wheel_torque':list(_native_snapshot_o.torque),
    'suspension_drives':{name:{a.GetName():str(a.Get()) for a in _native_snapshot_c.stage.GetPrimAtPath('/World/RC_Car/Joints/Suspension_'+name).GetAttributes() if a.GetName().startswith('drive:')} for name in ('FL','FR','RL','RR')},
},indent=2))

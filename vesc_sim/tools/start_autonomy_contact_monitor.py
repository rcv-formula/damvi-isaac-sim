from pathlib import Path as AutonomyMonitorFile
_autonomy_monitor_source=AutonomyMonitorFile('/home/shin/Desktop/RC_Car_Isaac/vesc_sim/tools/monitor_gui_contacts.py')
exec(compile(_autonomy_monitor_source.read_text(),str(_autonomy_monitor_source),'exec'),globals())
_damvi_contact_monitor.duration_s=90.
print('Contact guard extended to 90 simulated seconds for bounded autonomous validation.')

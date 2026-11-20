def _start_movie_contact_guard():
    from pathlib import Path
    source=Path.home()/'Desktop/RC_Car_Isaac/vesc_sim/tools/monitor_gui_contacts.py'
    namespace={}
    exec(compile(source.read_text(),str(source),'exec'),namespace)
    monitor=namespace['_damvi_contact_monitor']
    monitor.duration_s=175.
    output=Path.home()/'Desktop/RC_Car_Isaac/vesc_sim/outputs/dual_view_120s_20261113'
    output.mkdir(parents=True,exist_ok=True)
    monitor.output=output/'contact_check.json'
    return monitor
_damvi_movie_contact_monitor=_start_movie_contact_guard()

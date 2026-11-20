"""Executed by the trusted localhost Isaac Python server after a validation lap."""
def dump_episode_trace():
    import json
    import sys
    from pathlib import Path
    import live_gui

    c = live_gui.controller
    folder = Path(live_gui.__file__).resolve().parent
    # Direct loading also works with a server started before the trace tool was deployed.
    source = folder/'episode_trace.py'
    ns = {}
    exec(compile(source.read_bytes(), str(source), 'exec'), ns)
    trace = getattr(sys.modules.get('dataset_scene'), '_validation_trace', None)
    if trace is not None and trace.controller is c:
        trace.on_frame()
        rows = list(trace.rows if trace.started else trace.before)
        origin = 'episode_from_first_nonzero_target'
        receipts=list(getattr(trace,'command_receipts',[]))
    else:
        rows = [ns['encode_row'](r) for r in c.rows]
        origin = 'latest_40_seconds'
        receipts=[]
    reset_report = json.loads((c.output_dir/'dataset_reset.json').read_text())
    print(json.dumps(dict(dataset=c.config['_surface']['selected_dataset'],
        request_id=reset_report['request_id'], physics_hz=c.config['physics_hz'],
        columns=ns['COLUMNS'], source=origin, rows=rows,
        command_receipt_columns=['sim_s','executor_wall_ns','dds_source_wall_ns','dds_receive_wall_ns','speed','steer'],
        command_receipts=receipts), separators=(',',':')))


dump_episode_trace()

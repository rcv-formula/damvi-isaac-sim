"""Verify surface YAML reload in a running adapter, restoring the original file.

Run through isaac_live_client.py. No drive commands, poses or velocities are
written. The existing timeline is briefly paused and played to reload YAML.
"""


def _schedule_damvi_surface_reload_check():
    import asyncio
    import json
    from pathlib import Path
    import yaml
    import omni.kit.app
    import live_gui

    async def verify():
        c=live_gui.controller
        source=Path(c.config['_surface_path'])
        original=source.read_bytes()
        app=omni.kit.app.get_app()
        playing=c.timeline.is_playing()
        report={'before':c.config['_surface'],'status':'running'}
        try:
            candidate=yaml.safe_load(original)
            candidate['selected_dataset']='ifac2'
            c.timeline.pause();await app.next_update_async()
            source.write_text(yaml.safe_dump(candidate,sort_keys=False))
            c.timeline.play();await app.next_update_async();await app.next_update_async()
            report['temporary']=c.config['_surface']
            report['temporary_native_lateral_coefficients']=[p.friction_lateral for p in c.tires.model.params]
            if c.error:raise RuntimeError(c.error)
        except Exception as exc:
            report['error']=repr(exc)
        finally:
            c.timeline.pause();await app.next_update_async()
            source.write_bytes(original)
            c.timeline.play();await app.next_update_async();await app.next_update_async()
            report['restored']=c.config['_surface']
            report['restored_native_lateral_coefficients']=[p.friction_lateral for p in c.tires.model.params]
            temporary=report.get('temporary',{})
            report['checks']={
                'dataset_changed':temporary.get('selected_dataset')=='ifac2',
                'file_restored_exactly':source.read_bytes()==original,
                'profile_restored':report['restored']['selected_dataset']==report['before']['selected_dataset'],
                'one_coefficient_all_tires':all(abs(v-temporary.get('coefficient',-1))<1e-9
                    for v in report.get('temporary_native_lateral_coefficients',[])),
                'controller_healthy':c.error is None,
                'zero_target':c.target==0.,
            }
            report['status']='pass' if not report.get('error') and all(report['checks'].values()) else 'fail'
            (c.output_dir/'surface_reload_check.json').write_text(json.dumps(report,indent=2)+'\n')
            c.write_status()
            if not playing:c.timeline.pause()
            print('[SURFACE_RELOAD] '+json.dumps(report),flush=True)
    return asyncio.ensure_future(verify())


_damvi_surface_reload_task=_schedule_damvi_surface_reload_check()
print('Surface reload verification scheduled; original YAML will be restored.')

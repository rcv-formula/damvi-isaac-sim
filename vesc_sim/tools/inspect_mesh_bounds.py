def _mesh_bounds():
    import json,numpy as np,live_gui
    from pxr import UsdGeom
    c=live_gui.controller;stage=c.stage;cache=UsdGeom.XformCache();inv=cache.GetLocalToWorldTransform(stage.GetPrimAtPath(c.config['scene']['chassis'])).GetInverse()
    rows=[]
    for prim in stage.Traverse():
        path=str(prim.GetPath())
        if path.startswith(c.config['scene']['vehicle_root']) and prim.IsA(UsdGeom.Mesh):
            mesh=UsdGeom.Mesh(prim);points=np.asarray(mesh.GetPointsAttr().Get(),dtype=float)
            m=cache.GetLocalToWorldTransform(prim)*inv
            verts=np.array([m.Transform(tuple(v)) for v in points])
            rows.append({'path':path,'count':len(points),'lo':verts.min(axis=0).tolist(),'hi':verts.max(axis=0).tolist(),'centroid':verts.mean(axis=0).tolist(),'displayName':prim.GetDisplayName(),'customData':prim.GetCustomData()})
    print(json.dumps({'meshes':rows},indent=2))
_mesh_bounds()

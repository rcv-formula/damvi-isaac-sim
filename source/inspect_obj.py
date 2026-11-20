import sys, json, math, collections
from pathlib import Path
path = Path(sys.argv[1])
n = 0
f = 0
lo = [math.inf] * 3
hi = [-math.inf] * 3
groups = {}
group = '(default)'
materials = collections.Counter()
mtl = []
with path.open(encoding='utf-8', errors='replace') as stream:
    for line in stream:
        if line.startswith('v '):
            p = [float(v) for v in line.split()[1:4]]
            n += 1
            for i in range(3):
                lo[i] = min(lo[i], p[i])
                hi[i] = max(hi[i], p[i])
        elif line.startswith('f '):
            f += 1
            groups[group] = groups.get(group, 0) + 1
        elif line.startswith(('g ', 'o ')):
            group = line[2:].strip()
        elif line.startswith('usemtl '):
            materials[line[7:].strip()] += 1
        elif line.startswith('mtllib '):
            mtl.append(line[7:].strip())
print(json.dumps({'file': str(path), 'bytes': path.stat().st_size, 'vertices': n, 'faces': f, 'min': lo, 'max': hi, 'size': [hi[i] - lo[i] for i in range(3)], 'group_count': len(groups), 'groups': groups, 'material_count': len(materials), 'materials': dict(materials), 'mtllib': mtl}, indent=2))

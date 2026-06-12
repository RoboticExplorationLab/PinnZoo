URDF generated using the following command on commit hash 2a29691 from the [OpenArm description repo](https://github.com/enactic/openarm_description)
```
xacro $(ros2 pkg prefix openarm_description)/share/openarm_description/assets/robot/openarm_v2.0/urdf/openarm_v20.urdf.xacro   robot_preset:=default_bimanual > openarm_v20_bimanual.urdf
```

Dae meshes were converted to objs using trimesh and pycollada and this script
```
import trimesh
from pathlib import Path

# Point this to your actual meshes directory
mesh_dir = Path('meshes') 

# Recursively find all .dae files
for dae_file in mesh_dir.rglob('*.dae'):
    obj_file = dae_file.with_suffix('.obj')
    
    # Skip if we already converted it
    if obj_file.exists():
        continue
        
    print(f"Converting {dae_file.name} to .obj...")
    try:
        # force='mesh' merges complex Collada scenes into a single solid object
        mesh = trimesh.load(dae_file, force='mesh')
        mesh.export(obj_file)
    except Exception as e:
        print(f"Failed to convert {dae_file.name}: {e}")

print("Done converting meshes!")
```

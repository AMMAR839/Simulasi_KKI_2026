# Mesh Source

Source CAD:

`/home/ammar/Documents/asv_simulation/Final_TOT.3dm`

`final_tot_proxy.obj` is the active Gazebo-compatible mesh exported from
`Final_TOT.3dm` using:

`/home/ammar/Documents/asv_simulation/asv_kki_2026_ws/src/asv_description/scripts/convert_final_tot_3dm.py`

The converter reads Rhino Brep render meshes from the 3DM file, converts
millimeters to meters, recenters X/Y around the CAD bounding box, merges the
output into one OBJ object, and reduces face count for Gazebo stability. The
current exported mesh size is approximately:

- Length: 1.000 m
- Beam: 0.425 m
- Height: 0.761 m
- Active face count: about 87,360 faces from 1,083,457 source faces

Reference dimensions from Savinah One 2025 remain:

- LOA: 0.85014 m
- Beam: 0.420 m
- Height: 0.65949 m
- Draft: 0.110 m
- Displacement mass: 8.8 kg

The original `.3dm` file is never modified by this workspace.

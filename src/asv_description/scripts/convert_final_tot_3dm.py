#!/usr/bin/env python3
"""Export visible geometry from Final_TOT.3dm to Gazebo-friendly OBJ.

The Rhino file used for this workspace stores most of the boat as Brep objects
with render meshes attached to their faces. Gazebo cannot load 3DM directly, so
this script extracts existing Mesh objects and Brep face render meshes, converts
Rhino millimeters to meters, recenters X/Y around the CAD bounding box, and
writes an OBJ/MTL pair.
"""

import argparse
import sys
from pathlib import Path


DEFAULT_INPUT = Path("/home/ammar/Documents/asv_simulation/Final_TOT.3dm")
DEFAULT_OUTPUT = Path(
    "/home/ammar/Documents/asv_simulation/asv_kki_2026_ws/"
    "src/asv_description/models/asv_kki_2026/meshes/final_tot_from_3dm.obj"
)
ACTIVE_PROXY = Path(
    "/home/ammar/Documents/asv_simulation/asv_kki_2026_ws/"
    "src/asv_description/models/asv_kki_2026/meshes/final_tot_proxy.obj"
)
ACTIVE_MATERIAL = ACTIVE_PROXY.with_suffix(".mtl")
DEFAULT_MAX_FACES = 0


def unit_scale_to_meters(unit_system) -> float:
    name = str(unit_system).lower()
    if "millimeter" in name:
        return 0.001
    if "centimeter" in name:
        return 0.01
    if "meter" in name:
        return 1.0
    return 1.0


def iter_source_meshes(model, rhino3dm):
    for object_index, obj in enumerate(model.Objects):
        geom = obj.Geometry
        geom_type = geom.__class__.__name__
        if geom_type == "Mesh":
            yield f"mesh_object_{object_index}", geom
            continue
        if geom_type != "Brep":
            continue
        for face_index, face in enumerate(geom.Faces):
            mesh = None
            for mesh_type in (
                rhino3dm.MeshType.Render,
                rhino3dm.MeshType.Default,
                rhino3dm.MeshType.Any,
            ):
                mesh = face.GetMesh(mesh_type)
                if mesh is not None and len(mesh.Vertices) > 0 and len(mesh.Faces) > 0:
                    break
            if mesh is not None and len(mesh.Vertices) > 0 and len(mesh.Faces) > 0:
                yield f"brep_{object_index}_face_{face_index}", mesh


def mesh_bounds(meshes):
    mins = [float("inf"), float("inf"), float("inf")]
    maxs = [float("-inf"), float("-inf"), float("-inf")]
    for _, mesh in meshes:
        for vertex in mesh.Vertices:
            mins[0] = min(mins[0], vertex.X)
            mins[1] = min(mins[1], vertex.Y)
            mins[2] = min(mins[2], vertex.Z)
            maxs[0] = max(maxs[0], vertex.X)
            maxs[1] = max(maxs[1], vertex.Y)
            maxs[2] = max(maxs[2], vertex.Z)
    return mins, maxs


def write_material(path: Path):
    path.write_text(
        "\n".join(
            [
                "newmtl final_tot_cad",
                "Ka 0.56 0.70 0.02",
                "Kd 0.78 0.93 0.05",
                "Ks 0.35 0.38 0.18",
                "Ns 48.0",
                "d 1.0",
                "",
            ]
        ),
        encoding="utf-8",
    )


def material_path_for_output(output_path: Path) -> Path:
    if output_path == ACTIVE_PROXY.with_suffix(".obj.tmp"):
        return ACTIVE_MATERIAL
    return output_path.with_suffix(".mtl")


def face_indices(face):
    if hasattr(face, "IsTriangle"):
        if face.IsTriangle:
            return [face.A, face.B, face.C]
        return [face.A, face.B, face.C, face.D]
    values = list(face)
    if len(values) == 4 and values[2] == values[3]:
        return values[:3]
    return values


def mesh_faces_without_holes(mesh):
    """Return all mesh faces.

    Earlier versions reduced OBJ size by keeping only every Nth face. That made
    the CAD mesh visibly perforated in Gazebo. For the active boat visual we keep
    every face from the Rhino render mesh so the exported OBJ matches Final_TOT.
    """
    return list(range(len(mesh.Vertices))), [face_indices(face) for face in mesh.Faces]


def write_mesh_obj(input_path: Path, output_path: Path, max_faces: int) -> int:
    try:
        import rhino3dm
    except ImportError:
        print("rhino3dm is not installed. Install with: python3 -m pip install --user rhino3dm")
        return 2

    model = rhino3dm.File3dm.Read(str(input_path))
    if model is None:
        print(f"Could not read {input_path}")
        return 3

    meshes = list(iter_source_meshes(model, rhino3dm))
    if not meshes:
        skipped = {}
        for obj in model.Objects:
            geom_type = obj.Geometry.__class__.__name__
            skipped[geom_type] = skipped.get(geom_type, 0) + 1
        print("No mesh or Brep render-mesh geometry was exported from the 3dm file.")
        print(f"Skipped geometry types: {skipped}")
        print("Open the CAD model in Rhino/Blender/FreeCAD and export OBJ/DAE/STL.")
        return 4

    scale = unit_scale_to_meters(model.Settings.ModelUnitSystem)
    mins, maxs = mesh_bounds(meshes)
    center_x = (mins[0] + maxs[0]) * 0.5
    center_y = (mins[1] + maxs[1]) * 0.5
    size_m = [(maxs[i] - mins[i]) * scale for i in range(3)]

    source_face_count = sum(len(mesh.Faces) for _, mesh in meshes)
    if max_faces > 0 and source_face_count > max_faces:
        print(
            "Warning: --max-faces is kept for CLI compatibility, but unsafe "
            "face-skipping decimation is disabled because it creates holes. "
            "Exporting the full render mesh."
        )
    vertex_offset = 1
    face_count = 0
    vertex_count = 0
    output_path.parent.mkdir(parents=True, exist_ok=True)
    material_path = material_path_for_output(output_path)
    write_material(material_path)

    with output_path.open("w", encoding="utf-8") as handle:
        handle.write(f"# Exported from {input_path}\n")
        handle.write("# Rhino units converted to meters. X/Y centered on CAD bounding box.\n")
        handle.write(f"# Source unit system: {model.Settings.ModelUnitSystem}\n")
        handle.write(f"# Source bounds mm/native min: {mins}, max: {maxs}\n")
        handle.write(f"# Exported size meters: {size_m}\n")
        handle.write(f"# Source face count: {source_face_count}\n")
        handle.write("# Face stride: 1 (full render mesh; no hole-making face skipping)\n")
        handle.write(f"# Material file: {material_path.name}\n")
        handle.write(f"mtllib {material_path.name}\n")
        handle.write("o final_tot_cad_visual\n")
        handle.write("usemtl final_tot_cad\n")
        for name, mesh in meshes:
            vertex_indices, selected_faces = mesh_faces_without_holes(mesh)
            if not selected_faces:
                continue
            for index in vertex_indices:
                vertex = mesh.Vertices[index]
                x = (vertex.X - center_x) * scale
                y = (vertex.Y - center_y) * scale
                z = vertex.Z * scale
                handle.write(f"v {x:.9f} {y:.9f} {z:.9f}\n")
                vertex_count += 1
            for face in selected_faces:
                indices = [index + vertex_offset for index in face]
                handle.write("f " + " ".join(str(index) for index in indices) + "\n")
                face_count += 1
            vertex_offset += len(vertex_indices)

    print(f"Read {len(meshes)} source mesh section(s), {source_face_count} source faces.")
    print(f"Exported {vertex_count} vertices, {face_count} faces with full render mesh.")
    print(f"Model size after conversion: {size_m[0]:.3f} x {size_m[1]:.3f} x {size_m[2]:.3f} m")
    print(f"OBJ: {output_path}")
    print(f"MTL: {material_path}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--max-faces",
        type=int,
        default=DEFAULT_MAX_FACES,
        help=(
            "Deprecated compatibility option. The active exporter keeps all "
            "faces to avoid visual holes."
        ),
    )
    parser.add_argument(
        "--activate",
        action="store_true",
        help="Replace the active Gazebo proxy OBJ after a successful export.",
    )
    args = parser.parse_args()
    output = ACTIVE_PROXY.with_suffix(".obj.tmp") if args.activate else args.output
    result = write_mesh_obj(args.input, output, args.max_faces)
    if result == 0 and args.activate:
        output.replace(ACTIVE_PROXY)
        print(f"Activated converted mesh as {ACTIVE_PROXY}")
    return result


if __name__ == "__main__":
    sys.exit(main())

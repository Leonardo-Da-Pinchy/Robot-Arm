#!/usr/bin/env python3
"""Reconstruct the source CAD4 NoClaw proxy from its prescribed stock-part cut.

Requires trimesh and its cap triangulation dependency (included by yourdfpy's
extras in the CAD validation environment). Output is metres, matching stock STL.
The reconstruction is a mesh proxy, not a new SolidWorks part export.
"""
import json
from pathlib import Path
import trimesh


def main():
    package = Path(__file__).resolve().parents[1] / 'src/pinchy_description'
    mesh = trimesh.load(package / 'meshes/wrist_roll_follower_so101_v1.stl', force='mesh')
    # cad4_parameters.json: keep gripper z >= -37.05 mm == native part z <= 38 mm.
    mesh = mesh.slice_plane([0, 0, 0.038], [0, 0, -1], cap=True)
    expected = json.loads((package / 'cad/cad4/source/cad4_holder_measurements.json').read_text())
    volume = float(mesh.volume * 1e9)
    cad_volume = expected['follower_in_solidworks']['volume_mm3']
    if not mesh.is_watertight or abs(volume - cad_volume) > 10:
        raise ValueError('NoClaw mesh does not match expected closed-solid volume')
    output = package / 'meshes/cad4/wrist_roll_follower_so101_noclaw.stl'
    output.parent.mkdir(parents=True, exist_ok=True)
    mesh.export(output)
    report = dict(status='PASS', watertight=bool(mesh.is_watertight),
                  reconstructed_volume_mm3=volume, saved_cad_volume_mm3=cad_volume,
                  cut_native_part_z_m=0.038, mesh_units='metres',
                  note='Reconstructed CAD4 cut proxy; no surface-accuracy claim against an unavailable original mesh.')
    report_path = package.parents[1] / 'docs/validation/cad4/noclaw-mesh.json'
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, indent=2)+'\n')
    print(output)


if __name__ == '__main__':
    main()

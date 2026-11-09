#!/usr/bin/env python3
"""Tessellate supplied EE STEP files; preserve their native millimetre coordinates.

Optional build dependency: cadquery-ocp==7.9.3.1.1 (requires vtk==9.6.2).
These packages are not needed by the robot at runtime. No assembly transforms
or physical material densities are inferred by this conversion.
"""
from pathlib import Path

from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_Reader
from OCP.StlAPI import StlAPI_Writer


def main():
    package = Path(__file__).resolve().parents[1] / 'src/pinchy_description'
    output = package / 'meshes/ee_v1'
    output.mkdir(parents=True, exist_ok=True)
    for source in sorted((package / 'cad/EE_STEP_V1').glob('*.step')):
        reader = STEPControl_Reader()
        if reader.ReadFile(str(source)) != IFSelect_RetDone or not reader.TransferRoots():
            raise RuntimeError(f'Cannot read {source}')
        shape = reader.OneShape()
        mesher = BRepMesh_IncrementalMesh(shape, 0.08, False, 0.3, True)
        if not mesher.IsDone():
            raise RuntimeError(f'Tessellation failed: {source}')
        writer = StlAPI_Writer()
        writer.ASCIIMode = False
        destination = output / f'{source.stem}.stl'
        if not writer.Write(shape, str(destination)):
            raise RuntimeError(f'Cannot write {destination}')
        print(destination)


if __name__ == '__main__':
    main()

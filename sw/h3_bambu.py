"""h3_bambu.py -- a single-plate Bambu Studio project for one part, sliced.

    python sw/h3_bambu.py h3-tub      # -> exports/v3/plates/HOTARU3-tub-P1S.3mf
                                      #    exports/v3/plates/HOTARU3-tub-P1S.gcode.3mf

Settings are NOT invented here: Metadata/project_settings.config is copied
verbatim from HOTARU3-P1S-project.3mf (P1S 0.4, 0.20 mm Standard, Bambu PLA
Basic, Textured PEI, supports OFF), which the parts already printed used.
The part goes in with no per-object overrides, so it slices with supports
off. Bambu Studio's own CLI does the slicing.
"""
from __future__ import annotations

import os
import subprocess
import sys
import uuid
import zipfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import h3_stl as S

PLATES = os.path.join(os.path.dirname(S.DST), "plates")
SOURCE = os.environ.get("H3_SETTINGS_3MF", os.path.join(PLATES, "HOTARU3-P1S-project.3mf"))
BAMBU = r"D:\Program Files\Bambu Studio\bambu-studio.exe"
BED = 256.0


def build_project(name, out):
    t = S.read_stl(os.path.join(S.DST, name + ".stl"))
    v = t.reshape(-1, 3)
    uniq, inv = np.unique(np.round(v, 5), axis=0, return_inverse=True)
    F = inv.reshape(-1, 3)
    lo, hi = uniq.min(0), uniq.max(0)
    c = (lo + hi) / 2
    V = uniq - c                                    # Bambu stores meshes centred
    hz = (hi[2] - lo[2]) / 2
    vs = "\n".join(f'     <vertex x="{x:.6f}" y="{y:.6f}" z="{z:.6f}"/>' for x, y, z in V)
    fs = "\n".join(f'     <triangle v1="{a}" v2="{b}" v3="{d}"/>' for a, b, d in F)
    ns = ('xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02" '
          'xmlns:BambuStudio="http://schemas.bambulab.com/package/2021" '
          'xmlns:p="http://schemas.microsoft.com/3dmanufacturing/production/2015/06" '
          'requiredextensions="p"')
    obj = (f'<?xml version="1.0" encoding="UTF-8"?>\n<model unit="millimeter" xml:lang="en-US" {ns}>\n'
           ' <metadata name="BambuStudio:3mfVersion">1</metadata>\n <resources>\n'
           f'  <object id="1" p:UUID="{uuid.uuid4()}" type="model">\n   <mesh>\n    <vertices>\n'
           f'{vs}\n    </vertices>\n    <triangles>\n{fs}\n    </triangles>\n   </mesh>\n  </object>\n'
           ' </resources>\n <build/>\n</model>\n')
    model = (f'<?xml version="1.0" encoding="UTF-8"?>\n<model unit="millimeter" xml:lang="en-US" {ns}>\n'
             ' <metadata name="Application">BambuStudio-02.08.02.61</metadata>\n'
             ' <metadata name="BambuStudio:3mfVersion">1</metadata>\n'
             f' <metadata name="Title">HOTARU 3 {name}</metadata>\n <resources>\n'
             f'  <object id="2" p:UUID="{uuid.uuid4()}" type="model">\n   <components>\n'
             f'    <component p:path="/3D/Objects/object_1.model" objectid="1" p:UUID="{uuid.uuid4()}" '
             f'transform="1 0 0 0 1 0 0 0 1 0 0 {hz:.6f}"/>\n   </components>\n  </object>\n'
             ' </resources>\n <build>\n'
             f'  <item objectid="2" p:UUID="{uuid.uuid4()}" transform="1 0 0 0 1 0 0 0 1 '
             f'{BED / 2:.4f} {BED / 2:.4f} 0" printable="1"/>\n </build>\n</model>\n')
    msettings = ('<?xml version="1.0" encoding="UTF-8"?>\n<config>\n  <object id="2">\n'
                 f'    <metadata key="name" value="{name}.stl"/>\n'
                 '    <metadata key="extruder" value="1"/>\n'
                 f'    <metadata face_count="{len(F)}"/>\n'
                 '    <part id="1" subtype="normal_part">\n'
                 f'      <metadata key="name" value="{name}.stl"/>\n'
                 f'      <metadata key="matrix" value="1 0 0 0 0 1 0 0 0 0 1 {hz:.6f} 0 0 0 1"/>\n'
                 f'      <mesh_stat face_count="{len(F)}" edges_fixed="0" degenerate_facets="0" '
                 'facets_removed="0" facets_reversed="0" backwards_edges="0"/>\n'
                 '    </part>\n  </object>\n  <plate>\n'
                 '    <metadata key="plater_id" value="1"/>\n'
                 f'    <metadata key="plater_name" value="{name}"/>\n'
                 '    <metadata key="locked" value="false"/>\n'
                 '    <model_instance>\n      <metadata key="object_id" value="2"/>\n'
                 '      <metadata key="instance_id" value="0"/>\n'
                 '      <metadata key="identify_id" value="1"/>\n    </model_instance>\n'
                 '  </plate>\n  <assemble>\n'
                 '   <assemble_item object_id="2" instance_id="0" '
                 f'transform="1 0 0 0 1 0 0 0 1 {BED / 2:.4f} {BED / 2:.4f} {hz:.6f}" offset="0 0 0" />\n'
                 '  </assemble>\n</config>\n')
    src = zipfile.ZipFile(SOURCE)
    rels = ('<?xml version="1.0" encoding="UTF-8"?>\n<Relationships xmlns="http://schemas.openxmlformats.org/'
            'package/2006/relationships">\n <Relationship Target="/3D/Objects/object_1.model" Id="rel-1" '
            'Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>\n</Relationships>\n')
    with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("[Content_Types].xml", src.read("[Content_Types].xml"))
        z.writestr("_rels/.rels", src.read("_rels/.rels"))
        z.writestr("3D/3dmodel.model", model)
        z.writestr("3D/_rels/3dmodel.model.rels", rels)
        z.writestr("3D/Objects/object_1.model", obj)
        z.writestr("Metadata/project_settings.config", src.read("Metadata/project_settings.config"))
        z.writestr("Metadata/model_settings.config", msettings)
    return out


def slice_project(project, out):
    r = subprocess.run([BAMBU, "--slice", "0", "--export-3mf", out, project],
                       capture_output=True, text=True, timeout=1800)
    return r.returncode, (r.stdout or "")[-3000:], (r.stderr or "")[-3000:]


if __name__ == "__main__":
    name = sys.argv[1] if len(sys.argv) > 1 else "h3-tub"
    os.makedirs(PLATES, exist_ok=True)
    stem = name.replace("h3-", "")
    proj = build_project(name, os.path.join(PLATES, f"HOTARU3-{stem}-P1S.3mf"))
    print("project:", proj)
    code, out, err = slice_project(proj, os.path.join(PLATES, f"HOTARU3-{stem}-P1S.gcode.3mf"))
    print("bambu-studio exit", code)
    print(out[-1500:])
    print(err[-1500:])

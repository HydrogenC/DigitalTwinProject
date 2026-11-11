"""Tessellate every leaf part of the STEP assembly and write one OBJ per part.

Vertices are written in the STEP world frame (mm). Faces are grouped by colour
(usemtl c_RRGGBB). parts.json records each part's name, OBJ file, colour list
and bounding box, and the cylindrical faces (used later to find joint axes).
"""
import sys, os, json
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ColorType
from OCP.TDF import TDF_Label
from OCP.OCP.collections import Sequence_TDF_Label as TDF_LabelSequence
from OCP.TDataStd import TDataStd_Name
from OCP.IFSelect import IFSelect_RetDone
from OCP.TopLoc import TopLoc_Location
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
from OCP.TopoDS import TopoDS
from OCP.BRep import BRep_Tool
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder
from OCP.Quantity import Quantity_Color, Quantity_TOC_sRGB

step_path, out_dir = sys.argv[1], sys.argv[2]
os.makedirs(out_dir, exist_ok=True)

doc = TDocStd_Document(TCollection_ExtendedString("XmlOcaf"))
reader = STEPCAFControl_Reader()
reader.SetNameMode(True)
reader.SetColorMode(True)
if reader.ReadFile(step_path) != IFSelect_RetDone:
    sys.exit("read failed")
reader.Transfer(doc)
st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
ct = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())


def name_of(label):
    attr = TDataStd_Name()
    if label.FindAttribute(TDataStd_Name.GetID_s(), attr):
        return attr.Get().ToExtString()
    return "?"


def color_hex(c):
    r, g, b = c.Values(Quantity_TOC_sRGB)
    return "%02x%02x%02x" % (round(r * 255), round(g * 255), round(b * 255))


def shape_color(shape, fallback):
    c = Quantity_Color()
    for t in (XCAFDoc_ColorType.XCAFDoc_ColorSurf, XCAFDoc_ColorType.XCAFDoc_ColorGen):
        if ct.GetColor(shape, t, c):
            return color_hex(c)
    return fallback


def label_color(label, fallback):
    c = Quantity_Color()
    for t in (XCAFDoc_ColorType.XCAFDoc_ColorSurf, XCAFDoc_ColorType.XCAFDoc_ColorGen):
        if ct.GetColor_s(label, t, c):
            return color_hex(c)
    return fallback


parts = []


def export_leaf(inst_label, ref_label, loc, inherited_color):
    nauo = name_of(inst_label)
    proto = st.GetShape_s(ref_label)
    part_color = label_color(inst_label, label_color(ref_label, inherited_color))
    part_color = shape_color(proto, part_color)
    BRepMesh_IncrementalMesh(proto, 0.25, False, 0.35, True)
    trsf = loc.Transformation()

    by_color, cylinders = {}, []
    verts = []
    exp = TopExp_Explorer(proto, TopAbs_FACE)
    while exp.More():
        face = TopoDS.Face(exp.Current())
        exp.Next()
        col = shape_color(face, part_color)
        floc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, floc)
        if tri is not None:
            ftrsf = trsf.Multiplied(floc.Transformation())
            base = len(verts)
            for i in range(1, tri.NbNodes() + 1):
                p = tri.Node(i).Transformed(ftrsf)
                verts.append((p.X(), p.Y(), p.Z()))
            rev = face.Orientation() == TopAbs_REVERSED
            lst = by_color.setdefault(col, [])
            for i in range(1, tri.NbTriangles() + 1):
                a, b, c = tri.Triangle(i).Get()
                if rev:
                    b, c = c, b
                lst.append((base + a, base + b, base + c))
        surf = BRepAdaptor_Surface(face)
        if surf.GetType() == GeomAbs_Cylinder:
            ax = surf.Cylinder().Position().Transformed(trsf)
            o, d = ax.Location(), ax.Direction()
            cylinders.append([round(surf.Cylinder().Radius(), 2),
                              [round(o.X(), 3), round(o.Y(), 3), round(o.Z(), 3)],
                              [round(d.X(), 5), round(d.Y(), 5), round(d.Z(), 5)]])

    if not verts:
        return
    fname = f"{nauo}.obj"
    with open(os.path.join(out_dir, fname), "w") as f:
        for x, y, z in verts:
            f.write(f"v {x:.3f} {y:.3f} {z:.3f}\n")
        for col, tris in by_color.items():
            f.write(f"usemtl c_{col}\n")
            for a, b, c in tris:
                f.write(f"f {a} {b} {c}\n")
    lo = [min(v[i] for v in verts) for i in range(3)]
    hi = [max(v[i] for v in verts) for i in range(3)]
    # keep distinct cylinders only (one entry per axis+radius)
    uniq = {}
    for r, o, d in cylinders:
        uniq.setdefault((r, tuple(round(x, 1) for x in o), tuple(round(x, 3) for x in d)), [r, o, d])
    parts.append({"nauo": nauo, "part": name_of(ref_label), "obj": fname,
                  "colors": sorted(by_color), "verts": len(verts),
                  "tris": sum(len(t) for t in by_color.values()),
                  "bbox": [round(x, 2) for x in lo + hi],
                  "cylinders": list(uniq.values())})
    print(nauo, name_of(ref_label), len(verts), flush=True)


def walk(label, loc, color):
    ref = TDF_Label()
    if st.IsReference_s(label):
        st.GetReferredShape_s(label, ref)
        loc = loc * st.GetLocation_s(label)
    else:
        ref = label
    color = label_color(label, label_color(ref, color))
    if st.IsAssembly_s(ref):
        kids = TDF_LabelSequence()
        st.GetComponents_s(ref, kids, False)
        for i in range(1, kids.Length() + 1):
            walk(kids.Value(i), loc, color)
    else:
        export_leaf(label, ref, loc, color)


free = TDF_LabelSequence()
st.GetFreeShapes(free)
for i in range(1, free.Length() + 1):
    walk(free.Value(i), TopLoc_Location(), "b4b4b4")

with open(os.path.join(out_dir, "parts.json"), "w", encoding="utf-8") as f:
    json.dump(parts, f, ensure_ascii=False, indent=1)
print("parts:", len(parts))

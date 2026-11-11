"""Print the assembly tree of a STEP file: names, instance transforms, bounding boxes."""
import sys, json
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TDF import TDF_Label
from OCP.OCP.collections import Sequence_TDF_Label as TDF_LabelSequence
from OCP.TDataStd import TDataStd_Name
from OCP.IFSelect import IFSelect_RetDone
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.TopLoc import TopLoc_Location

path = sys.argv[1]
out_json = sys.argv[2]

doc = TDocStd_Document(TCollection_ExtendedString("XmlOcaf"))
reader = STEPCAFControl_Reader()
reader.SetNameMode(True)
reader.SetColorMode(True)
if reader.ReadFile(path) != IFSelect_RetDone:
    sys.exit("read failed")
reader.Transfer(doc)
st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())


def name_of(label):
    attr = TDataStd_Name()
    if label.FindAttribute(TDataStd_Name.GetID_s(), attr):
        return attr.Get().ToExtString()
    return "?"


def trsf_to_list(loc):
    t = loc.Transformation()
    return [[t.Value(r, c) for c in range(1, 5)] for r in range(1, 4)]


def bbox(shape):
    b = Bnd_Box()
    BRepBndLib.Add_s(shape, b)
    if b.IsVoid():
        return None
    lo, hi = b.CornerMin(), b.CornerMax()
    return [round(v, 1) for v in (lo.X(), lo.Y(), lo.Z(), hi.X(), hi.Y(), hi.Z())]


def walk(label, loc, depth, out):
    """label: an instance (reference) or a top-level shape."""
    ref = TDF_Label()
    inst_name = name_of(label)
    if st.IsReference_s(label):
        st.GetReferredShape_s(label, ref)
        loc = loc * st.GetLocation_s(label)
    else:
        ref = label
    node = {"name": inst_name, "part": name_of(ref), "world": trsf_to_list(loc), "children": []}
    shape = st.GetShape_s(ref).Moved(loc)
    node["bbox"] = bbox(shape)
    if st.IsAssembly_s(ref):
        kids = TDF_LabelSequence()
        st.GetComponents_s(ref, kids, False)
        for i in range(1, kids.Length() + 1):
            walk(kids.Value(i), loc, depth + 1, node["children"])
    out.append(node)


free = TDF_LabelSequence()
st.GetFreeShapes(free)
tree = []
for i in range(1, free.Length() + 1):
    walk(free.Value(i), TopLoc_Location(), 0, tree)

with open(out_json, "w", encoding="utf-8") as f:
    json.dump(tree, f, ensure_ascii=False, indent=1)


def show(n, d, f):
    org = [round(n["world"][r][3], 1) for r in range(3)]
    f.write("  " * d + f"{n['name']} [{n['part']}] org={org} bbox={n['bbox']}\n")
    for c in n["children"]:
        if d < 3 or "FR3" in n["part"]:
            show(c, d + 1, f)


with open(out_json.replace(".json", ".txt"), "w", encoding="utf-8") as f:
    for n in tree:
        show(n, 0, f)


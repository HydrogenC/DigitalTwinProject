"""Offscreen preview of the exported OBJ parts (STEP frame, up = -Z).

usage: render.py obj_dir out.png focal_x,y,z cam_dir_x,y,z distance [nauo,nauo,...]
"""
import sys, os, json
import vtk

obj_dir, out = sys.argv[1], sys.argv[2]
focal = [float(v) for v in sys.argv[3].split(",")]
cdir = [float(v) for v in sys.argv[4].split(",")]
dist = float(sys.argv[5])
only = set(sys.argv[6].split(",")) if len(sys.argv) > 6 else None
parts = json.load(open(os.path.join(obj_dir, "parts.json"), encoding="utf-8"))

ren = vtk.vtkRenderer()
ren.SetBackground(0.92, 0.93, 0.95)
for p in parts:
    if only and p["nauo"] not in only:
        continue
    # split by usemtl so each colour gets its own actor
    groups, cur = {}, None
    verts = []
    for line in open(os.path.join(obj_dir, p["obj"])):
        if line[0] == "v":
            verts.append([float(x) for x in line.split()[1:]])
        elif line[0] == "u":
            cur = line.split()[1][2:]
        elif line[0] == "f":
            groups.setdefault(cur, []).append([int(x) - 1 for x in line.split()[1:]])
    pts = vtk.vtkPoints()
    for v in verts:
        pts.InsertNextPoint(v)
    for col, faces in groups.items():
        cells = vtk.vtkCellArray()
        for f in faces:
            cells.InsertNextCell(3, f)
        pd = vtk.vtkPolyData()
        pd.SetPoints(pts)
        pd.SetPolys(cells)
        nm = vtk.vtkPolyDataNormals()
        nm.SetInputData(pd)
        m = vtk.vtkPolyDataMapper()
        m.SetInputConnection(nm.GetOutputPort())
        a = vtk.vtkActor()
        a.SetMapper(m)
        a.GetProperty().SetColor(*[int(col[i:i + 2], 16) / 255 for i in (0, 2, 4)])
        ren.AddActor(a)

axes = vtk.vtkAxesActor()
axes.SetTotalLength(100, 100, 100)
axes.SetPosition(*focal)
ren.AddActor(axes)

cam = ren.GetActiveCamera()
cam.SetFocalPoint(*focal)
n = sum(c * c for c in cdir) ** 0.5
cam.SetPosition(*[focal[i] + cdir[i] / n * dist for i in range(3)])
cam.SetViewUp(0, 0, -1)
cam.SetViewAngle(30)
ren.ResetCameraClippingRange()

win = vtk.vtkRenderWindow()
win.SetOffScreenRendering(1)
win.AddRenderer(ren)
win.SetSize(1100, 850)
win.Render()
w2i = vtk.vtkWindowToImageFilter()
w2i.SetInput(win)
w2i.Update()
wr = vtk.vtkPNGWriter()
wr.SetFileName(out)
wr.SetInputConnection(w2i.GetOutputPort())
wr.Write()

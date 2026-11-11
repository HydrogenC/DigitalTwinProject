"""Convert exported STEP parts to Unity space and describe the scene for the editor builder.

STEP frame: mm, right-handed, up = -Z, table spans X[-1000,0] Y[0,1000].
Unity frame: m, left-handed, up = +Y, table centred at the origin:
    unity = ((X + 500), -Z, -(Y - 500)) / 1000
The mapping is a reflection, so triangle winding is reversed.

All joint axes, pivots and frames below were measured from the CAD geometry
(coaxial cylinder faces shared by neighbouring links, hinge pin, pad holes).
"""
import os, json, sys
import numpy as np

src_dir, out_dir = sys.argv[1], sys.argv[2]
os.makedirs(os.path.join(out_dir, "meshes"), exist_ok=True)
parts = {p["nauo"]: p for p in json.load(open(os.path.join(src_dir, "parts.json"), encoding="utf-8"))}


def P(p):  # STEP point -> Unity point
    X, Y, Z = p
    return [round((X + 500) / 1000, 6), round(-Z / 1000, 6), round(-(Y - 500) / 1000, 6)]


def D(d):  # STEP direction -> Unity direction (normalised)
    X, Y, Z = d
    v = np.array([X, -Z, -Y], float)
    return [round(float(x), 6) for x in v / np.linalg.norm(v)]


def norm(v):
    v = np.array(v, float)
    return v / np.linalg.norm(v)


# ---------------------------------------------------------------- groups
groups = {
    "Table": ["NAUO52", "NAUO53", "NAUO54"],
    "RobotBase": ["NAUO45", "NAUO18", "NAUO21", "NAUO22", "NAUO23", "NAUO24", "NAUO25", "NAUO26", "NAUO27",
                  "NAUO30", "NAUO31", "NAUO32", "NAUO33", "NAUO34", "NAUO35", "NAUO36", "NAUO37", "NAUO38", "NAUO39"],
    "Link1": ["NAUO46"],
    "Link2": ["NAUO47"],
    "Link3": ["NAUO48"],
    "Link4": ["NAUO49"],
    "Link5": ["NAUO50"],
    "Link6": ["NAUO51", "NAUO19", "NAUO42", "NAUO40", "NAUO41", "NAUO28", "NAUO29"],
    "JawA": ["NAUO43", "NAUO17"],
    "JawB": ["NAUO44", "NAUO16"],
    "ColumnSupport": ["NAUO3"],
    "TiltSupportBase": ["NAUO8", "NAUO11", "NAUO13"],
    "TiltSupportTop": ["NAUO9", "NAUO12"],
    "Drone": ["NAUO6", "NAUO7"],
}

# ---------------------------------------------------------------- meshes
for g, ids in groups.items():
    for n in ids:
        verts, out = [], []
        for line in open(os.path.join(src_dir, parts[n]["obj"])):
            t = line.split()
            if not t:
                continue
            if t[0] == "v":
                u = P([float(x) for x in t[1:4]])
                out.append(f"v {u[0]:.6f} {u[1]:.6f} {u[2]:.6f}\n")
            elif t[0] == "usemtl":
                out.append(line)
            elif t[0] == "f":
                out.append(f"f {t[1]} {t[3]} {t[2]}\n")  # reflection flips winding
        with open(os.path.join(out_dir, "meshes", f"{n}.obj"), "w") as f:
            f.writelines(out)

# ---------------------------------------------------------------- robot
# axis point / direction of each joint in the CAD (reference) pose
joints_step = [
    ([-500.0, 720.0, -20.6], [0, 0, -1]),
    ([-446.2, 718.3, -159.4], [-0.9995, 0.0318, 0]),
    ([-332.3, 716.1, -439.4], [-0.9995, 0.0318, 0]),
    ([-548.1, 724.1, -679.4], [0.9995, -0.0318, 0]),
    ([-398.2, 717.0, -626.2], [-0.0014, -0.0448, 0.999]),
    ([-400.0, 762.7, -575.4], [0.0355, -0.9984, -0.0447]),
]
joints = [{"point": P(p), "axis": D(d)} for p, d in joints_step]

# tool: approach along J6 axis; TCP on the axis between the finger pads
a6 = norm(joints_step[5][1])
q6 = np.array(joints_step[5][0])
t = (q6[1] - 503.0) / -a6[1]
tcp_step = q6 + a6 * t
jaw_step = np.array([-333.78, 507.0, -586.8]) - np.array([-447.18, 503.0, -586.9])  # pad A -> pad B
jaw_step = norm(jaw_step - (jaw_step @ a6) * a6)
tool = {"tcp": P(tcp_step), "approach": D(a6), "jaw": D(jaw_step),
        "openGap": 0.1134, "maxStroke": 0.012}

# ---------------------------------------------------------------- supports
# pad frames: origin = pad top centre, up = pad normal, right = pad long side (u), forward = v
tilt0 = np.degrees(np.arccos(0.95157))           # current hinge angle of the CAD model
tilt_pad = {"origin": P([-380.178, 301.733, -100.637]),
            "up": D([-0.30743, 0, -0.95157]), "right": D([0.95157, 0, -0.30743])}
column_pad = {"origin": P([-638.686, 305.796, -107.5]), "up": D([0, 0, -1]), "right": D([1, 0, 0])}
supports = {
    "column": {"center": P([-638.686, 305.796, -1.5]), "pad": column_pad},
    "tilt": {"center": P([-368.341, 300.733, -1.5]), "pad": tilt_pad,
             "hingePoint": P([-368.341, 300.733, -64.0]), "hingeAxis": D([0, 1, 0]),
             "modelTiltDeg": round(float(tilt0), 4)},
}

# drone grasp point (in pad frame: u, w = height above pad top, v)
drone = {"graspInPad": {"right": -0.0007, "up": 0.010, "forward": 0.0}}

scene = {"groups": groups, "joints": joints, "tool": tool, "supports": supports, "drone": drone,
         "parts": {n: {"name": parts[n]["part"], "colors": parts[n]["colors"]} for ids in groups.values() for n in ids}}
with open(os.path.join(out_dir, "scene.json"), "w", encoding="utf-8") as f:
    json.dump(scene, f, ensure_ascii=False, indent=1)


# JsonUtility-friendly copy (no dictionaries, vectors as {x,y,z}) for the Unity editor builder
def V(v):
    return {"x": float(v[0]), "y": float(v[1]), "z": float(v[2])}


tp, cp = supports["tilt"], supports["column"]
unity_scene = {
    "groups": [{"name": g, "parts": ids} for g, ids in groups.items()],
    "parts": [{"id": n, "name": parts[n]["part"]} for ids in groups.values() for n in ids],
    "joints": [{"point": V(j["point"]), "axis": V(j["axis"])} for j in joints],
    "tcp": V(tool["tcp"]), "approach": V(tool["approach"]), "jaw": V(tool["jaw"]),
    "openGap": tool["openGap"], "maxStroke": tool["maxStroke"],
    "columnCenter": V(cp["center"]), "columnPadOrigin": V(cp["pad"]["origin"]),
    "columnPadUp": V(cp["pad"]["up"]), "columnPadRight": V(cp["pad"]["right"]),
    "tiltCenter": V(tp["center"]), "tiltPadOrigin": V(tp["pad"]["origin"]),
    "tiltPadUp": V(tp["pad"]["up"]), "tiltPadRight": V(tp["pad"]["right"]),
    "hingePoint": V(tp["hingePoint"]), "hingeAxis": V(tp["hingeAxis"]), "modelTiltDeg": tp["modelTiltDeg"],
    "graspInPad": V([drone["graspInPad"]["right"], drone["graspInPad"]["up"], drone["graspInPad"]["forward"]]),
}
with open(os.path.join(out_dir, "scene_unity.json"), "w", encoding="utf-8") as f:
    json.dump(unity_scene, f, ensure_ascii=False, indent=1)
print("tcp", tool["tcp"], "jaw", tool["jaw"], "approach", tool["approach"], "tilt0", tilt0)

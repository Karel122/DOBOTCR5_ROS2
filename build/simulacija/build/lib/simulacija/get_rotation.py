import math

# ---- your pasted values ----
tocke_raw = [
    [0.23957438738931286, -0.703621212827048, 0.2497962283808288, 0.0, 0.0, 0.0],
    [0.501604717397408, -1.116223520842319, 0.3456244538925689, 0.0, 0.0, 0.0],
    [0.04124490738361347, -1.0039171447530872, 0.36907251129415664, 0.0, 0.0, 0.0],
    # ... keep the rest of your 48 rows ...
    [-5.810454245136487e-06, -1.4695969489570166, -0.17659823662071217, 0.0, 0.0, 0.0],
]

# ==========================================================
# Choose what rx,ry,rz should be for ALL points
# (Degrees. Change these.)
# Examples:
#   Tool-down guess: rx=180, ry=0, rz=0
#   No rotation:     rx=0,   ry=0, rz=0
# ==========================================================
DEFAULT_RX = 180.0
DEFAULT_RY = 0.0
DEFAULT_RZ = 0.0

def to_xyz_rpy(tocke_raw, rx=DEFAULT_RX, ry=DEFAULT_RY, rz=DEFAULT_RZ):
    """
    Input:  [[x,y,z,*,*,*], ...]  (your list)
    Output: [[x,y,z,rx,ry,rz], ...]
    rx,ry,rz are set to constants because the input has no orientation info.
    """
    out = []
    for p in tocke_raw:
        x, y, z = float(p[0]), float(p[1]), float(p[2])
        out.append([x, y, z, float(rx), float(ry), float(rz)])
    return out

tocke_xyz_rpy = to_xyz_rpy(tocke_raw)

# print or use it
print("tocke_xyz_rpy[0] =", tocke_xyz_rpy[0])
print("count =", len(tocke_xyz_rpy))

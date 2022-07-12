import enum
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import mpl_toolkits.mplot3d.art3d as art3d
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial.transform import Rotation
import utils
from matplotlib.colors import to_rgba
from matplotlib.patches import ArrowStyle



utils.initRCParams()
fig = plt.figure()
ax = fig.add_subplot(projection='3d')



ARM_LENGTH = 0.2
ROTOR_HEIGHT = 0.1
ROTOR_RADIUS = 0.12
ROTOR_ANGLES_DEG = [45.0, 135.0, 225.0, 315.0]

drone_x = 0.0
drone_y = 0.0
drone_z = 0.0
drone_p = np.array([
    drone_x, drone_y, drone_z
])

body_sx = 0.2
body_sy = 0.15
body_sz = 0.08
drone_body_s = np.array([
    body_sx, body_sy, body_sz
])

def rotZ (a: float):
    sa = np.sin(a)
    ca = np.cos(a)
    return np.array([
        [ca, -sa, 0.0],
        [sa, ca, 0.0],
        [0.0, 0.0, 1.0]
    ])

def rotor_pos (
    a: float, # angle in degree
    al: float, # arm length
    rh: float # rotor height
):
    return rotZ(utils.rad__deg(a)) @ np.array([al, 0.0, rh])

rotor_p = []
for ra in ROTOR_ANGLES_DEG:
    rotor_p.append(
        rotor_pos(ra, ARM_LENGTH, ROTOR_HEIGHT)
    )

for rp in rotor_p:
    print(rp)
    p = Circle(
        tuple(rp[:2]), 
        ROTOR_RADIUS,
        color='k',
        alpha=0.4
    )
    ax.add_patch(p)
    art3d.pathpatch_2d_to_3d(p, z=rp[2], zdir="z")

    ax.plot(
        [drone_p[0], rp[0]],
        [drone_p[1], rp[1]],
        [drone_p[2], rp[2]],
        color='k',
        alpha=0.5,
        linewidth=3
    )
#
#ax.scatter(
#    *list(drone_p),
#    color='k',
#    alpha=0.5
#)








def unitFace(
    axis: str, 
    shift: float
) -> np.ndarray:
    vert = np.zeros((3))
    verts = np.zeros((4, 3))
    if axis == 'x': vert[0]=shift;  i0 = 1;         i1 = 2
    if axis == 'y': i0 = 0;         vert[1]=shift;  i1 = 2
    if axis == 'z': i0 = 0;         i1 = 1;         vert[2]=shift
    vert[i0] = -0.5; vert[i1] = -0.5; verts[0] = vert
    vert[i0] = 0.5; vert[i1] = -0.5; verts[1] = vert
    vert[i0] = 0.5; vert[i1] = 0.5; verts[2] = vert
    vert[i0] = -0.5; vert[i1] = 0.5; verts[3] = vert
    return verts


def quaternion__from__rotationAngleAxis(angle: float, axis : np.ndarray):
    return np.array([
        np.cos(angle/2.0),
        np.sin(angle/2.0) * axis[0],
        np.sin(angle/2.0) * axis[1],
        np.sin(angle/2.0) * axis[2]
    ])
def multiplyQuaternions(q0: np.ndarray, q1: np.ndarray):
    w0 = q0[0]; xyz0 = q0[1:]
    w1 = q1[0]; xyz1 = q1[1:]
    w = w0*w1 - np.inner(xyz0, xyz1)
    xyz = w0*xyz1 + w1*xyz0 + np.cross(xyz0, xyz1)
    return np.array([w, xyz[0], xyz[1], xyz[2]])
def invertQuaternion(q: np.ndarray):
    return np.array([q[0], -q[1], -q[2], -q[3]]) / np.linalg.norm(q)
def rotateVectorByQuaternion(xyz: np.ndarray, q: np.ndarray):
    wxyz = np.array([0, xyz[0], xyz[1], xyz[2]])
    return multiplyQuaternions(multiplyQuaternions(q,wxyz), invertQuaternion(q))[1:]
def radian__from__degree(degree: float):
    return degree * np.pi / 180.0

def fovData(
    aov_y: float,
    aov_z: float,
    d1: float,
    d2: float,
    x: float = 0,
    y: float = 0,
    z: float = 0,
    ax: float = 0,
    ay: float = 0,
    az: float = 0,
) -> np.ndarray:
    p = np.array([x, y, z])
    ex = np.array([1.0, 0, 0])
    ey = np.array([0, 1.0, 0])
    ez = np.array([0, 0, 1.0])

    x1 = d1 * ex
    x2 = d2 * ex

    q_l = quaternion__from__rotationAngleAxis(angle=aov_z / 2, axis=ez)
    q_r = quaternion__from__rotationAngleAxis(angle=- aov_z / 2, axis=ez)
    q_u = quaternion__from__rotationAngleAxis(angle=- aov_y / 2, axis=ey)
    q_d = quaternion__from__rotationAngleAxis(angle=aov_y / 2, axis=ey)

    q_lu = multiplyQuaternions(q_l, q_u)
    q_ru = multiplyQuaternions(q_r, q_u)
    q_rd = multiplyQuaternions(q_r, q_d)
    q_ld = multiplyQuaternions(q_l, q_d)

    p1_lu = rotateVectorByQuaternion(x1, q_lu)
    p1_ru = rotateVectorByQuaternion(x1, q_ru)
    p1_rd = rotateVectorByQuaternion(x1, q_rd)
    p1_ld = rotateVectorByQuaternion(x1, q_ld)
    p2_lu = rotateVectorByQuaternion(x2, q_lu)
    p2_ru = rotateVectorByQuaternion(x2, q_ru)
    p2_rd = rotateVectorByQuaternion(x2, q_rd)
    p2_ld = rotateVectorByQuaternion(x2, q_ld)

    q_x = quaternion__from__rotationAngleAxis(angle=ax, axis=ex)
    q_y = quaternion__from__rotationAngleAxis(angle=ay, axis=ey)
    q_z = quaternion__from__rotationAngleAxis(angle=az, axis=ez)
    q_xyz = multiplyQuaternions(multiplyQuaternions(q_x, q_y), q_z)

    p1_lu = rotateVectorByQuaternion(p1_lu, q_xyz) + p
    p1_ru = rotateVectorByQuaternion(p1_ru, q_xyz) + p
    p1_rd = rotateVectorByQuaternion(p1_rd, q_xyz) + p
    p1_ld = rotateVectorByQuaternion(p1_ld, q_xyz) + p
    p2_lu = rotateVectorByQuaternion(p2_lu, q_xyz) + p
    p2_ru = rotateVectorByQuaternion(p2_ru, q_xyz) + p
    p2_rd = rotateVectorByQuaternion(p2_rd, q_xyz) + p
    p2_ld = rotateVectorByQuaternion(p2_ld, q_xyz) + p


    return np.array([
        [p1_lu, p1_ru, p1_rd, p1_ld], #p1
        [p2_lu, p2_ru, p2_rd, p2_ld], #p2
        [p1_lu, p1_ru, p2_ru, p2_lu], #u
        [p1_ru, p1_rd, p2_rd, p2_ru], #r
        [p1_ld, p1_rd, p2_rd, p2_ld], #d
        [p1_lu, p1_ld, p2_ld, p2_lu] #l
    ])


    

    
    



def cuboidData (
    x: float, 
    y: float, 
    z: float,
    sx: float, 
    sy: float, 
    sz: float
) -> np.ndarray:
    X = np.array([
       unitFace(axis='x', shift=-0.5),
       unitFace(axis='y', shift=-0.5),
       unitFace(axis='z', shift=-0.5),
       unitFace(axis='x', shift=0.5),
       unitFace(axis='y', shift=0.5),
       unitFace(axis='z', shift=0.5),
    ], dtype=np.float)
    for i, s in enumerate([sx, sy, sz]):
        X[:, :, i] *= s
    print(X)
    return X + np.array([x, y, z])






ax.add_collection(
    Poly3DCollection(
        cuboidData(
            drone_x, drone_y, drone_z,
            body_sx, body_sy, body_sz
        ),
        color='k',
        alpha=0.2
    )
)

ax.add_collection(
    Poly3DCollection(
        fovData(
            1*1.05 * np.pi/4,
            1*np.pi/4,
            0.3,
            10,
            x=0,
            y=0,
            z=0,
            ax=radian__from__degree(0),
            ay=radian__from__degree(0),
            az=radian__from__degree(0)
        ),
        color='tab:orange',
        alpha=0.1,
        edgecolor=to_rgba('tab:orange', 0.2),
        #label='Camera FOV'
    )
)


length = 0.5

ax.arrow3D(
    drone_x, drone_y, drone_z,
    drone_x + length, drone_y, drone_z,
    mutation_scale=20,
    arrowstyle=ArrowStyle.CurveFilledB(head_length=.4, head_width=.1),
    linewidth=3,
    color='tab:red'
)
ax.arrow3D(
    drone_x, drone_y, drone_z,
    drone_x, drone_y + length, drone_z,
    mutation_scale=20,
    arrowstyle=ArrowStyle.CurveFilledB(head_length=.4, head_width=.1),
    linewidth=3,
    color='tab:red'
)
ax.arrow3D(
    drone_x, drone_y, drone_z,
    drone_x, drone_y, drone_z + length,
    mutation_scale=20,
    arrowstyle=ArrowStyle.CurveFilledB(head_length=.4, head_width=.1),
    linewidth=3,
    color='tab:red'
)
ax.annotate3D(
    '${}_\\textbf{L} e^\\text{L}_x$', 
    (drone_x + length, drone_y, drone_z), 
    xytext=(0, 0), 
    textcoords='offset points',
    color='tab:red'
)
ax.annotate3D(
    '${}_\\textbf{L} e^\\text{L}_y$', 
    (drone_x, drone_y + length, drone_z), 
    xytext=(0, 0), 
    textcoords='offset points',
    color='tab:red'
)
ax.annotate3D(
    '${}_\\textbf{L} e^\\text{L}_z$', 
    (drone_x, drone_y, drone_z + length), 
    xytext=(0, 0), 
    textcoords='offset points',
    color='tab:red'
)


auxUnitBox = 0.5 * np.array([
    [-1, -1, -1],
    [-1, -1,  1],
    [-1,  1, -1],
    [-1,  1,  1],
    [ 1, -1, -1],
    [ 1, -1,  1],
    [ 1,  1, -1],
    [ 1,  1,  1],
])

auxBox = auxUnitBox + 0.5
ax.scatter(
    auxBox[:, 0],
    auxBox[:, 1],
    auxBox[:, 2],
    alpha=0
)

plt.axis('off')
ax.set_xlim(-1, 1)
ax.set_ylim(-1, 1)
ax.set_zlim(-0.8, 1.2)

#utils.set_axes_equal(ax)

# Functions from @Mateen Ulhaq and @karlo
def set_axes_equal(ax: plt.Axes):
    """Set 3D plot axes to equal scale.

    Make axes of 3D plot have equal scale so that spheres appear as
    spheres and cubes as cubes.  Required since `ax.axis('equal')`
    and `ax.set_aspect('equal')` don't work on 3D.
    """
    limits = np.array([
        ax.get_xlim3d(),
        ax.get_ylim3d(),
        ax.get_zlim3d(),
    ])
    origin = np.mean(limits, axis=1)
    radius = 0.5 * np.max(np.abs(limits[:, 1] - limits[:, 0]))
    _set_axes_radius(ax, origin, radius)

def _set_axes_radius(ax, origin, radius):
    x, y, z = origin
    ax.set_xlim3d([x - radius, x + radius])
    ax.set_ylim3d([y - radius, y + radius])
    ax.set_zlim3d([z - radius, z + radius])

#ax.set_box_aspect([1,1,1])
set_axes_equal(ax)


#plt.grid()
#plt.show()

ax.view_init(45, 180+45)
ax.dist = 2

ax.legend()

utils.saveFig(__file__)
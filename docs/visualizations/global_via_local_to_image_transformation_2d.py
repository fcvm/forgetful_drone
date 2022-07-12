from matplotlib.lines import Line2D
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrow
import pathlib
import math
import plot_utils


### Plot configuration
plt.rcParams['text.usetex'] = True
plt.rcParams['text.latex.preamble'] = [
    r'\usepackage{amssymb}',
    r'\usepackage{amsmath}',
    r'\usepackage{mathtools}'
]

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



### Parameters
# geometry
g_droneXYZ = np.array([2.1, 2, 2])
g_droneRotAngle = radian__from__degree(-20.0)
g_droneRotAxis = np.array([0, 0, 1]);

drone_arm_len = 0.3
drone_rotor_radius = 0.17
halfyawAOV = 40.0
halfpitchAOV = 30.0
drone_camera_vis_shift = 0.8

g_Waypoint = np.array([4.8, 2.2, 2])
#g_Waypoint = np.array([4.8, -1, 2])

# plot
arrowHeadSize=0.1
RFlinewidth=3
RFalpha=0.6
AUXalpha=0.5
g_color = 'tab:green'
l_color = 'tab:red'
i_color = 'tab:blue'
drone_arm_color = 'k'
drone_arm_alpha = 0.4
drone_rotor_color = 'k'
drone_rotor_alpha = 0.2
drone_cam_fov_arc_radius = 3.7

width_rf = 0.005
width_value = 0.0035





### Constants
origin = np.array([0, 0, 0])
eX = np.array([1,0,0])
eY = np.array([0,1,0])
eZ = np.array([0,0,1])



# Computations
g_g0 = origin
g_geX = eX
g_geY = eY
g_geZ = eZ


g_droneRotAxis = g_droneRotAxis / np.linalg.norm(g_droneRotAxis)
g_DroneQuat = quaternion__from__rotationAngleAxis(
    angle=g_droneRotAngle, 
    axis=g_droneRotAxis
)
def T_g_l(l_p):
    return rotateVectorByQuaternion(xyz=l_p, q=g_DroneQuat) + g_droneXYZ
def T_l_g(g_p):
    return rotateVectorByQuaternion(
        xyz=g_p - g_droneXYZ, 
        q=invertQuaternion(g_DroneQuat)
    )


g_l0 = T_g_l(origin)
g_leX = T_g_l(eX)
g_leY = T_g_l(eY)
g_leZ = T_g_l(eZ)
g_leX100 = T_g_l(100 * eX)


# Drone
l_arm_ = [
    drone_arm_len * np.array([
        np.cos(radian__from__degree(x)), 
        np.sin(radian__from__degree(x)), 0
    ]) for x in [45.0, 135.0, 225.0, 315.0]
]
g_arm_ = [
    T_g_l(x) for x in l_arm_
]

# FOV
l_fovL = drone_camera_vis_shift * np.array([
    np.cos(radian__from__degree(halfyawAOV)), 
    np.sin(radian__from__degree(halfyawAOV)), 
    0
])
l_fovR = drone_camera_vis_shift * np.array([
    np.cos(radian__from__degree(-halfyawAOV)), 
    np.sin(radian__from__degree(-halfyawAOV)), 
    0
])
l_fovL100 = 100 * l_fovL
l_fovR100 = 100 * l_fovR
g_fovL = T_g_l(l_fovL)
g_fovR = T_g_l(l_fovR)
g_fovL100 = T_g_l(l_fovL100)
g_fovR100 = T_g_l(l_fovR100)

g_i0 = np.array([
    (g_fovL[0] + g_fovR[0]) / 2,
    (g_fovL[1] + g_fovR[1]) / 2
])
g_ieX = np.array([
    g_fovR[0],
    g_fovR[1]
])


# Waypoint
l_Waypoint = T_l_g(g_Waypoint)
l_yaw2wayp = math.atan2(l_Waypoint[1], l_Waypoint[0])
print(l_yaw2wayp)
l_pitch2wayp = math.atan2(l_Waypoint[2], np.linalg.norm(l_Waypoint))

def T_i_l(i_p):
    return np.array([
        np.max([
            -1, 
            np.min([
                1, 
                - math.atan2(
                    i_p[1], 
                    i_p[0]
                ) / plot_utils.rad__from__deg(halfyawAOV)
            ])
        ]),
        np.max([
            -1, 
            np.min([
                1, 
                math.atan2(
                    i_p[2], 
                    np.linalg.norm(i_p)
                ) / plot_utils.rad__from__deg(halfpitchAOV)
            ])
        ])
    ])


i_wayp = T_i_l(T_l_g(g_Waypoint))










#################################
############# PLOT
#############################
fig, ax = plt.subplots()


# DRONE
for xy in [(g_arm_[0], g_arm_[2]), (g_arm_[1], g_arm_[3])]:
    ax.plot(
        [xy[0][0], xy[1][0]], 
        [xy[0][1], xy[1][1]], 
        c=drone_arm_color,
        alpha=drone_arm_alpha
    )
for xy in g_arm_:
    ax.add_patch(
        plt.Circle(
            (xy[0], xy[1]), 
            drone_rotor_radius,
            color=drone_rotor_color, 
            alpha=drone_rotor_alpha
        )
    )

# FOV
ax.fill(
    [g_fovL[0], g_fovL100[0], g_fovR100[0], g_fovR[0]], 
    [g_fovL[1], g_fovL100[1], g_fovR100[1], g_fovR[1]],
    color='tab:orange', alpha=0.1, label='Camera FOV')

ax.plot(
    [g_leX[0], g_leX100[0]],
    [g_leX[1], g_leX100[1]],
    linestyle=':',
    color=l_color,
    alpha=AUXalpha
)
plot_utils.annotatedAngle(
    ax=ax,
    aux1xy1xy2=(tuple(g_leX[:2]), tuple((g_leX100)[:2])),
    aux2xy1xy2=(tuple(g_fovL[:2]), tuple(g_fovL100[:2])),
    radius=drone_cam_fov_arc_radius,
    text='$\\frac{1}{2}\\phi^\\text{user}_{z,\\text{camera}}$',
    textdxdy=(0, 2),
    width=width_value,
    color=l_color,
    alpha=1
)





# GLOBAL REFERENCE SYSTEM (GRF)
plot_utils.annotatedVector(
    ax=ax,
    xy1=tuple(g_g0[:2]),
    xy2=tuple(g_geX[:2]),
    text='${}_\\textbf{G} \\underline e_x^\\text{G}$',
    textdxdy=(0, 0),
    width=width_rf,
    color=g_color,
    alpha=RFalpha
)

plot_utils.annotatedVector(
    ax=ax,
    xy1=tuple(g_g0[:2]),
    xy2=tuple(g_geY[:2]),
    text='${}_\\textbf{G} \\underline e_y^\\text{G}$',
    textdxdy=(0, 0),
    width=width_rf,
    color=g_color,
    alpha=RFalpha
)




# DRONE POSITION (GRF)
plot_utils.annotatedVector(
    ax=ax,
    xy1=tuple(g_g0[:2]),
    xy2=tuple(g_droneXYZ[:2]),
    text='${}_\\textbf{G} \\underline p^\\text{drone}$',
    textdxdy=(-35, -5),
    width=width_value,
    color=g_color,
    alpha=1
)


# DRONE ORIENTATION (GRF)
ax.plot(
    [g_l0[0], g_l0[0] + g_geY[0]],
    [g_l0[1], g_l0[1] + g_geY[1]],
    linestyle=':',
    color=g_color,
    alpha=AUXalpha
)

plot_utils.annotatedAngle(
    ax=ax,
    aux1xy1xy2=(tuple(g_l0[:2]), tuple((g_l0 + g_geY)[:2])),
    aux2xy1xy2=(tuple(g_l0[:2]), tuple(g_leY[:2])),
    radius=3/4,
    text='${}_\\textbf{G} \\underline q^\\text{drone}$',
    textdxdy=(2, -10),
    width=width_value,
    color=g_color,
    alpha=1
)



# WAYPOINT [GRF]
plot_utils.annotatedVector(
    ax=ax,
    xy1=tuple(g_g0[:2]),
    xy2=tuple(g_Waypoint[:2]),
    text='${}_\\textbf{G} \\underline p^\\text{wayp}$',
    textdxdy=(-2, -15),
    width=width_value,
    color=g_color,
    alpha=1
)



# LOCAL REFERENCE SYSTEM (LRF)
plot_utils.annotatedVector(
    ax=ax,
    xy1=tuple(g_l0[:2]),
    xy2=tuple(g_leX[:2]),
    text='${}_\\textbf{L} \\underline e_x^\\text{L}$',
    textdxdy=(0, 0),
    width=width_rf,
    color=l_color,
    alpha=RFalpha
)
plot_utils.annotatedVector(
    ax=ax,
    xy1=tuple(g_l0[:2]),
    xy2=tuple(g_leY[:2]),
    text='${}_\\textbf{L} \\underline e_y^\\text{L}$',
    textdxdy=(0, 0),
    width=width_rf,
    color=l_color,
    alpha=RFalpha
)

# WAYPOINT [LRF]
plot_utils.annotatedVector(
    ax=ax,
    xy1=tuple(g_l0[:2]),
    xy2=tuple(g_Waypoint[:2]),
    text='${}_\\textbf{L} \\underline p^\\text{wayp}$',
    textdxdy=(-2, 5),
    width=width_value,
    color=l_color,
    alpha=1
)


# YAW TO WAYPOINT (LRF)
plot_utils.annotatedAngle(
    ax=ax,
    aux1xy1xy2=(tuple(g_l0[:2]), tuple(g_leX[:2])),
    aux2xy1xy2=(tuple(g_l0[:2]), tuple(g_Waypoint[:2])),
    radius=drone_cam_fov_arc_radius/2,
    text='${}_\\textbf{L}\\phi^\\text{wayp}_{z}$',
    textdxdy=(-5, 2),
    width=0.0035,
    color=l_color,
    alpha=1
)


# IMAGE REFERENCE SYSTEM (IRF)
plot_utils.annotatedVector(
    ax=ax,
    xy1=tuple(g_i0[:2]),
    xy2=tuple(g_ieX[:2]),
    text='${}_\\textbf{I} \\underline e_x^\\text{I}$',
    textdxdy=(-13, -3),
    width=width_rf,
    color=i_color,
    alpha=RFalpha
)



# WAYPOINT [IRF]
plot_utils.annotatedVector(
    ax=ax,
    xy1=tuple(g_i0[:2]),
    xy2=tuple((g_i0 + i_wayp[0] * (g_ieX - g_i0))[:2]),
    text='${}_\\textbf{I} \\underline p^\\text{wayp}$',
    textdxdy=(2, 1),
    width=width_value,
    color=i_color,
    alpha=1
)























mpl.rcParams['legend.fontsize'] = 10
ax.legend(loc='lower right')

ax.set_aspect('equal',adjustable='box')

#ax.set_xticks([])
#ax.set_yticks([])
plt.axis('off')
ax.set_xlim([-0.1, 6.5])
ax.set_ylim([-0.1, 3.6])

plt.savefig(
    pathlib.Path(__file__).parent.resolve()/f"{pathlib.Path(__file__).stem}.pdf", 
    bbox_inches='tight',
    #bbox_inches=ax.get_window_extent().transformed(fig.dpi_scale_trans.inverted()),
    pad_inches=0
)
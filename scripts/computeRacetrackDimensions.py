import matplotlib.pyplot as plt
from matplotlib.patches import Arc
import numpy as np
from typing import List, Tuple




class Pos:

    @staticmethod
    def fromXYZ (x: float, y: float, z: float): 
        return np.array ([x, y, z], dtype=np.float)

    @staticmethod
    def assertPos (p: np.ndarray) -> None:
        if not p.shape == (3,): raise ValueError (f"v.shape: {p.shape} != (3,)")

    
class Quat:

    @staticmethod
    def assertFloat (x: float) -> None:
        if not type (x) == float: raise ValueError (f"type (x): {type (x)} != float")
    def assertVec (v: np.ndarray) -> None:
        if not v.shape == (3,): raise ValueError (f"v.shape: {v.shape} != (3,)")
    def assertQuat (q: np.ndarray) -> None:
        if not q.shape == (4,): raise ValueError (f"q.shape: {q.shape} != (4,)")

    @staticmethod
    def fromWXYZ (w: float, x: float, y: float, z: float) -> np.ndarray:
        Quat.assertFloat (w)
        Quat.assertFloat (x)
        Quat.assertFloat (y)
        Quat.assertFloat (z)
        return np.array ([w, x, y, z], dtype=np.float)

    @staticmethod
    def fromAngAx (ang: float, ax : np.ndarray) -> np.ndarray:
        Quat.assertFloat (ang)
        Quat.assertVec (ax)
        return np.array ([
            np.cos (ang/2.0),
            np.sin (ang/2.0) * ax [0],
            np.sin (ang/2.0) * ax [1],
            np.sin (ang/2.0) * ax [2]
        ], dtype=np.float)

    @staticmethod
    def fromVec (v: np.ndarray) -> np.ndarray:
        Quat.assertVec (v)
        return np.array ([0, v [0], v [1], v [2]], dtype=np.float)

    @staticmethod
    def multiply (q1: np.ndarray, q2: np.ndarray):
        Quat.assertQuat (q1)
        Quat.assertQuat (q2)
        w1 = q1 [0]; p1 = q1 [1:]
        w2 = q2 [0]; p2 = q2 [1:]
        w = w1 * w2 - np.inner (p1, p2)
        p = w1 * p2 + w2 * p1 + np.cross (p1, p2)
        return np.array ([w, p [0], p [1], p [2]])

    @staticmethod
    def invert (q: np.ndarray):
        Quat.assertQuat (q)
        return np.array ([q [0], -q [1], -q [2], -q [3]]) / np.linalg.norm (q)
    

    @staticmethod
    def rotateVec (v: np.ndarray, q: np.ndarray):
        Quat.assertVec (v)
        Quat.assertQuat (q)
        return Quat.multiply (
            Quat.multiply (
                q, 
                Quat.fromVec (v)
            ), 
            Quat.invert (q)
        ) [1:]

    @staticmethod
    def yaw (q: np.ndarray) -> float:
        Quat.assertQuat (q)
        return np.arctan2 (
            2.0 * (q [3] * q [0] + q [1] * q [2]), 
            - 1.0 + 2.0 * (q [0] * q [0] + q [1] * q [1])
        )

    @staticmethod 
    def unitAng (q : np.ndarray, ax : int) -> float:
        if ax not in [0, 1, 2]: raise ValueError (f"Axis index: {ax} != 0, 1, 2")
        return 2 * np.arctan2 (q [1 + ax], q [0])
    
   
class Pose:
    def __init__ (self, p : np.ndarray = None, q : np.ndarray = None) -> None:
        if p is None : p = Pos.fromXYZ (0.0, 0.0, 0.0)
        if q is None : q = Quat.fromWXYZ (0.0, 0.0, 0.0, 0.0)
        Pos.assertPos (p)
        Quat.assertQuat (q)
        self.p = p
        self.q = q



class Racetrack: 
    def __init__ (self, gate_poses : List [Pose]) -> None:
        self.gateWidth = 3.35
        self.gateMinHeight = 3.7
        self.gateConstYawTwist = - np.pi / 2
        self.frontGateIdx = -2
        
        self.randMaxScale = 1.2
        self.randMinScale = 0.8
        self.randMaxAxShift = 0.8
        self.randMaxYawTwist = 5.0 / 180 * np.pi

        self.gatePosesDet = gate_poses
        self.dronePoseDet = self.compDronePose (self.gatePosesDet)

        self.gatePosesRand = self.randTrack (self.gatePosesDet)
        self.dronePoseRand = self.compDronePose (self.gatePosesRand)

    def compDronePose (self, gate_poses : List [Pose]) -> Pose:
        front = gate_poses [self.frontGateIdx] 
        back = gate_poses [self.frontGateIdx - 1]
        
        drone = Pose ()

        drone.q = Quat.multiply (
            front.q,
            Quat.fromAngAx (self.gateConstYawTwist, UnitVecZ)
        )
        
        drone.p = (front.p + back.p) / 2
        ldv = np.array ([
            np.cos (Quat.unitAng (front.q, 2) - self.gateConstYawTwist),
            np.sin (Quat.unitAng (front.q, 2) - self.gateConstYawTwist),
        ])
        opm = np.outer (ldv, ldv) / np.inner (ldv, ldv)
        p0 = np.array ([front.p [0], front.p [1]])
        p = np.array ([drone.p [0], drone.p [1]])
        drone.p [: 2] = opm @ (p - p0) + p0

        return drone
    
    def getTrackSize (self, gate_poses : List [Pose]) -> Tuple:
        
        for i in range (len (gate_poses)):
            x = self.gatePosesDet [i].p [0]
            y = self.gatePosesDet [i].p [1]
            z = self.gatePosesDet [i].p [2]

            if i == 0:
                xmin = xmax = x
                ymin = ymax = y
                zmin = zmax = z
            
            else:
                if x < xmin: xmin = x
                if y < ymin: ymin = y
                if z < zmin: zmin = z

                if x > xmax: xmax = x
                if y > ymax: ymax = y
                if z > zmax: zmax = z
        
        xmin -= self.gateWidth 
        ymin -= self.gateWidth 
        zmin -= self.gateWidth 
        xmax += self.gateWidth 
        ymax += self.gateWidth 
        zmax += self.gateWidth 
        
        return ([xmin, xmax], [ymin, ymax], [zmin, zmax])

    def plotTrackXY (self, ax, gate_poses : List [Pose], drone_pose : Pose) -> None:
        # gates
        for i in range (len (gate_poses)):
            x = gate_poses [i].p [0]
            y = gate_poses [i].p [1]
            yaw = Quat.yaw (gate_poses [i].q)
            lbl = i

            dx = np.cos (yaw) * self.gateWidth / 2 
            dy = np.sin (yaw) * self.gateWidth / 2 
            ax.arrow (x - dx, y - dy, 2 * dx, 2 * dy, 
                width=0.3, head_width=0.0, color="grey"
            )

            #ax.add_patch(
            #    plt.Circle ((x, y), 0.3, color='black', clip_on=False, fill=True)
            #)

            ax.annotate (lbl, (x + 1.0, y + 1.0))

        # drone
        dx = np.cos (Quat.yaw (drone_pose.q)) * self.gateWidth / 200
        dy = np.sin (Quat.yaw (drone_pose.q)) * self.gateWidth / 200
        ax.arrow (
            drone_pose.p [0], 
            drone_pose.p [1], 
            dx,
            dy, 
            width=0.3, head_width=2, color="blue", overhang=.3
        )
        ax.annotate ('Init. drone', (drone_pose.p [0], drone_pose.p [1]))

            

        
        #xticks = ax.get_xticks()
        #yticks = ax.get_yticks()
        #if (xticks[-1] > yticks[-1]):
        #    ticks_stepsize = yticks[1] - yticks[0]
        #    ax.set_xticks(np.arange(xticks[0], xticks[-1], ticks_stepsize))
        #else:
        #    ticks_stepsize = xticks[1] - xticks[0]
        #    ax.set_yticks(np.arange(yticks[0], yticks[-1], ticks_stepsize))
        #ax.grid()
        ax.set_aspect('equal', adjustable='box')
        xlim, ylim, _ = self.getTrackSize (gate_poses)
        ax.set_xlim (xlim)
        ax.set_ylim (ylim)
        

    def randTrack (self, gate_poses : List [Pose]) -> List [Pose]:        
        scale = np.random.uniform (self.randMinScale, self.randMaxScale)

        rand_poses = []
        for p in gate_poses:
            shift = self.randMaxAxShift * np.random.uniform (-1.0, 1.0, (3,))
            twist = self.randMaxYawTwist * np.random.uniform (-1.0, 1.0)
        
            rp = Pose ()
            rp.p = (p.p + shift) * scale
            rp.q = Quat.multiply (p.q, Quat.fromAngAx (twist, UnitVecZ))
            rand_poses.append (rp)

        return rand_poses


        






UnitVecZ = np.array ([0, 0, 1.0])
Fig8DetX = [-20.45, -12.55, -4.15, 3.45, 11.95, 21.85, 24.25, 19.25, 10.55, 2.85, -4.95, -12.95, -21.05, -24.25]
Fig8DetY = [-8.65, -11.15, -5.35, 4.25, 11.15, 6.85, -1.75, -9.55, -10.65, -5.95, 4.65, 9.65, 6.65, -1.00]
Fig8DetZ = [2.0] * len (Fig8DetX)
Fig8DetYaw = [1.13, -1.57, -0.60, -0.63, -1.21, 0.99, 0.00, -1.03, 1.53, 0.57, 0.67, -1.53, -0.77, 0.07,]
 

Fig8DetPoses = []
for x, y, z, yaw in zip (Fig8DetX, Fig8DetY, Fig8DetZ, Fig8DetYaw):
    Fig8DetPoses.append (Pose (
        Pos.fromXYZ (x, y, z),
        Quat.fromAngAx (yaw, UnitVecZ)
    ))

RTFig8 = Racetrack (Fig8DetPoses)

fig, ax = plt.subplots (1, 1)
RTFig8.plotTrackXY (ax, RTFig8.gatePosesDet, RTFig8.dronePoseDet)
fig.tight_layout()
plt.show()




RAND_MaxScale = 1.2

#%% Data

# gate dimensions (unity prefab) (ignoring the feet)
gate_width = 3.35
gate_minHeight = 3.7




# gate positions in deterministic figure 8 racetrack (rpg code) (last element: drone starting position)
RPG_GatesX = np.array([2.7 , 10.6, 19.0, 26.6, 35.1, 45.0, 47.4, 42.4, 33.7, 26.0, 18.2, 10.2, 2.1 , -1.1, -0.5])
RPG_GatesY = np.array([6.7 , 4.2 , 10.0, 19.6, 26.5, 22.2, 13.6, 5.8 , 4.7 , 9.4 , 20.0, 25.0, 22.0, 13.2, 22.0])
RPG_GatesYaw = np.array([-0.44, 0.0  , 0.97 , -2.2 , 3.5  , 2.57 , 1.57 , -2.6 , 3.1  , -1.0 , -0.9 , -3.1 , 0.8  , -1.5, 0]) -np.pi/2




#%% my deterministic racetrack data
det_GatesX = RPG_GatesX
det_GatesY = RPG_GatesY
GatesYaw = RPG_GatesYaw






# ->center gates around origin
det_GatesX = det_GatesX - np.mean(RPG_GatesX[:-1])
det_GatesY = det_GatesY - np.mean(RPG_GatesY[:-1])

# ->compute provisional racetrack dimensions
det_RacetrackDimX = max(det_GatesX) - min(det_GatesX)
det_RacetrackDimY = max(det_GatesY) - min(det_GatesY)

# ->center minimum, x-/y-axis aligned recktangle, that encloses racetrack, around origin
if (np.max(det_GatesX) > np.max(-det_GatesX)):
    det_GatesX = det_GatesX - (np.max(det_GatesX) - det_RacetrackDimX/2)
else:
    det_GatesX = det_GatesX + (np.max(-det_GatesX) - det_RacetrackDimX/2)
if (np.max(det_GatesY) > np.max(-det_GatesY)):
    det_GatesY = det_GatesY - (np.max(det_GatesY) - det_RacetrackDimY/2)
else:
    det_GatesY = det_GatesY + (np.max(-det_GatesY) - det_RacetrackDimY/2)

# compute racetrack dimensions (minimum, x-/y-axis aligned recktangle, that encloses racetrack)
#RacetrackDimX = RacetrackDimX + RPG_GateWidth + 2*RAND_MaxAxialShift
#RacetrackDimY = RacetrackDimY + RPG_GateWidth + 2*RAND_MaxAxialShift
det_RacetrackDimX = det_RacetrackDimX + gate_width
det_RacetrackDimY = det_RacetrackDimY + gate_width


np.set_printoptions(precision=2)
print("Deterministic Figure-8 Racetrack Gate poses:")
print("\tx:", det_GatesX)
print("\ty:", det_GatesY)
print("\tyaw:", GatesYaw)

print("Minimum, x-/y-axis aligned rectangle enclosing race track [DET.]")
print("\tx:", det_RacetrackDimX)
print("\ty:", det_RacetrackDimY)
print("\tz:", gate_minHeight)



#%% my randomized racetrack data

# parameters for randomizing
RAND_MaxAxialShift = 0.8
RAND_MinScale = 0.8
RAND_MaxScale = 1.2


# Min sized racetrack
min_GatesMinX = (det_GatesX - RAND_MaxAxialShift) * RAND_MinScale
min_GatesMaxX = (det_GatesX + RAND_MaxAxialShift) * RAND_MinScale
min_RacetrackDimX = max(min_GatesMaxX) - min(min_GatesMinX) + gate_width

min_GatesMinY = (det_GatesY - RAND_MaxAxialShift) * RAND_MinScale
min_GatesMaxY = (det_GatesY + RAND_MaxAxialShift) * RAND_MinScale
min_RacetrackDimY = max(min_GatesMaxY) - min(min_GatesMinY) + gate_width

print("Minimum, x-/y-axis aligned rectangle enclosing race track [MIN. RAND.]")
print("\tx:", min_RacetrackDimX)
print("\ty:", min_RacetrackDimY)
print("\tz:", (gate_minHeight+2*RAND_MaxAxialShift)*RAND_MinScale)


# Max sized racetrack
max_GatesMinX = (det_GatesX - RAND_MaxAxialShift) * RAND_MaxScale
max_GatesMaxX = (det_GatesX + RAND_MaxAxialShift) * RAND_MaxScale
max_RacetrackDimX = max(max_GatesMaxX) - min(max_GatesMinX) + gate_width

max_GatesMinY = (det_GatesY - RAND_MaxAxialShift) * RAND_MaxScale
max_GatesMaxY = (det_GatesY + RAND_MaxAxialShift) * RAND_MaxScale
max_RacetrackDimY = max(max_GatesMaxY) - min(max_GatesMinY) + gate_width

print("Minimum, x-/y-axis aligned rectangle enclosing race track [MIN. RAND.]")
print("\tx:", max_RacetrackDimX)
print("\ty:", max_RacetrackDimY)
print("\tz:", (gate_minHeight+2*RAND_MaxAxialShift)*RAND_MaxScale)




#%% Space for obstacles

det_ObstX = np.array([-det_RacetrackDimX/4*1.1, det_RacetrackDimX/4, 0, -det_RacetrackDimX/64])
det_ObstY = np.array([0, 0, det_RacetrackDimY/1.8, -det_RacetrackDimY/1.75])
det_ObstR = np.array([5, 5, 5, 5])

min_ObstX = det_ObstX * RAND_MinScale
min_ObstY = det_ObstY * RAND_MinScale
min_ObstR = det_ObstR * RAND_MinScale

max_ObstX = det_ObstX * RAND_MaxScale
max_ObstY = det_ObstY * RAND_MaxScale
max_ObstR = det_ObstR * RAND_MaxScale





#%% Plot Deterministic

plot_labels = ["1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14", "Start"]
gate_radius = gate_width/2
fig, ax = plt.subplots(1, 1)


# for every gate
for i in range(np.size(det_GatesX) -1):

    x = det_GatesX[i]; y = det_GatesY[i]; yaw = GatesYaw[i]; label = plot_labels[i]
    
    # ->plot gate from bird's eye view
    dx = np.cos(yaw) * gate_radius; dy = np.sin(yaw) * gate_radius
    ax.arrow(x, y, dx, dy, width=0.3, head_width=0.0, color="grey"); ax.arrow(x, y, -dx, -dy, width=0.3, head_width=0.0, color="grey")

    # ->plot gate center (=waypoint)
    ax.add_patch(plt.Circle((x, y), 0.3, color='black', clip_on=False, fill=True))

    # ->annotate 
    ax.annotate(label, (x + 1.0, y + 1.0))

# -plot drone starting pose
plt.arrow(det_GatesX[-1], det_GatesY[-1], np.cos(GatesYaw[-1])*gate_radius/100, np.sin(GatesYaw[-1])*gate_radius/100, width=0.3, head_width=2, color="blue", overhang=.3)
plt.annotate(plot_labels[-1], (det_GatesX[-1], det_GatesY[-1]))


# ->plot minimum, x-/y-axis aligned rectangle enclosing race track
ax.add_patch(plt.Rectangle(
    (-det_RacetrackDimX/2, -det_RacetrackDimY/2),
    det_RacetrackDimX,  det_RacetrackDimY,   
    color="black", fill=False, linestyle="--"))

# -plot obstacle area
for i in range(np.size(det_ObstR)):
    ax.add_patch(plt.Circle((det_ObstX[i], det_ObstY[i]), det_ObstR[i], color='pink', clip_on=True, fill=True))



# ->add quadratic grid
xticks = ax.get_xticks()
yticks = ax.get_yticks()
if (xticks[-1] > yticks[-1]):
    ticks_stepsize = yticks[1] - yticks[0]
    ax.set_xticks(np.arange(xticks[0], xticks[-1], ticks_stepsize))
else:
    ticks_stepsize = xticks[1] - xticks[0]
    ax.set_yticks(np.arange(yticks[0], yticks[-1], ticks_stepsize))
ax.grid()
ax.set_aspect('equal', adjustable='box')
fig.tight_layout()
plt.show()



#%% Plot Minimum

fig, ax = plt.subplots(1, 1)


# for every gate
for i in range(np.size(det_GatesX) -1):

    min_x = min_GatesMinX[i]; max_x = min_GatesMaxX[i]
    min_y = min_GatesMinY[i]; max_y = min_GatesMaxY[i]
    label = plot_labels[i]
    
    # ->annotate 
    ax.annotate(label, ((min_x+max_x)/2, (min_y+max_y)/2))

    # ->plot gate axial shift area
    ax.add_patch(plt.Rectangle(
        (min_x, min_y),
        max_x - min_x, max_y -min_y,
        color="black", fill=False, linestyle="--"))

    # ->plot area that may be occupied by gate (due to yawing and shifting)
    ax.plot(
        [min_x, max_x],
        [max_y + gate_radius, max_y + gate_radius],
        color='green', linestyle='dashed', linewidth=1
        )
    ax.plot(
        [min_x, max_x],
        [min_y - gate_radius, min_y - gate_radius],
        color='green', linestyle='dashed', linewidth=1
        )
    ax.plot(
        [max_x + gate_radius, max_x + gate_radius],
        [min_y, max_y],
        color='green', linestyle='dashed', linewidth=1
        )
    ax.plot(
        [min_x - gate_radius, min_x - gate_radius],
        [min_y, max_y],
        color='green', linestyle='dashed', linewidth=1
        )

    arc_x = np.array([max_x, max_x, min_x, min_x])
    arc_y = np.array([max_y, min_y, max_y, min_y])
    arc_theta1 = [0.0, 270.0, 90.0, 180.0]
    arc_theta2 = [90.0, 360.0, 180.0, 270.0]
    for j in range(np.size(arc_x)):
        ax.add_patch(Arc(
                (arc_x[j], arc_y[j]),
                gate_width, gate_width,
                theta1=arc_theta1[j], theta2=arc_theta2[j],
                color='green', linestyle='dashed', linewidth=1, clip_on=False, fill=False))



# ->plot drone starting pose area
min_x = min_GatesMinX[-1]; max_x = min_GatesMaxX[-1]
min_y = min_GatesMinY[-1]; max_y = min_GatesMaxY[-1]
ax.add_patch(plt.Rectangle(
    (min_x, min_y),
    max_x - min_x, max_y - min_y,
    color="black", fill=False, linestyle="--"))
plt.arrow((min_x + max_x)/2, (min_y + max_y)/2, np.cos(GatesYaw[-1])*gate_radius/100, np.sin(GatesYaw[-1])*gate_radius/100, width=0.3, head_width=2, color="blue", overhang=.3)
plt.annotate(plot_labels[-1], ((min_x + max_x)/2, (min_y + max_y)/2))


# ->plot minimum, x-/y-axis aligned rectangle enclosing race track
ax.add_patch(plt.Rectangle(
    (-min_RacetrackDimX/2, -min_RacetrackDimY/2),
    min_RacetrackDimX,  min_RacetrackDimY,   
    color="black", fill=False, linestyle="--"))

# -plot obstacle area
for i in range(np.size(det_ObstR)):
    ax.add_patch(plt.Circle((min_ObstX[i], min_ObstY[i]), min_ObstR[i], color='pink', clip_on=True, fill=True))

# ->add quadratic grid
xticks = ax.get_xticks()
yticks = ax.get_yticks()
if (xticks[-1] > yticks[-1]):
    ticks_stepsize = yticks[1] - yticks[0]
    ax.set_xticks(np.arange(xticks[0], xticks[-1], ticks_stepsize))
else:
    ticks_stepsize = xticks[1] - xticks[0]
    ax.set_yticks(np.arange(yticks[0], yticks[-1], ticks_stepsize))
ax.grid()
ax.set_aspect('equal', adjustable='box')
fig.tight_layout()
plt.show()







#%% Plot Maximum

fig, ax = plt.subplots(1, 1)


# for every gate
for i in range(np.size(det_GatesX) -1):

    min_x = max_GatesMinX[i]; max_x = max_GatesMaxX[i]
    min_y = max_GatesMinY[i]; max_y = max_GatesMaxY[i]
    label = plot_labels[i]
    
    # ->annotate 
    ax.annotate(label, ((min_x+max_x)/2, (min_y+max_y)/2))

    # ->plot gate axial shift area
    ax.add_patch(plt.Rectangle(
        (min_x, min_y),
        max_x - min_x, max_y -min_y,
        color="black", fill=False, linestyle="--"))

    # ->plot area that may be occupied by gate (due to yawing and shifting)
    ax.plot(
        [min_x, max_x],
        [max_y + gate_radius, max_y + gate_radius],
        color='green', linestyle='dashed', linewidth=1
        )
    ax.plot(
        [min_x, max_x],
        [min_y - gate_radius, min_y - gate_radius],
        color='green', linestyle='dashed', linewidth=1
        )
    ax.plot(
        [max_x + gate_radius, max_x + gate_radius],
        [min_y, max_y],
        color='green', linestyle='dashed', linewidth=1
        )
    ax.plot(
        [min_x - gate_radius, min_x - gate_radius],
        [min_y, max_y],
        color='green', linestyle='dashed', linewidth=1
        )

    arc_x = np.array([max_x, max_x, min_x, min_x])
    arc_y = np.array([max_y, min_y, max_y, min_y])
    arc_theta1 = [0.0, 270.0, 90.0, 180.0]
    arc_theta2 = [90.0, 360.0, 180.0, 270.0]
    for j in range(np.size(arc_x)):
        ax.add_patch(Arc(
                (arc_x[j], arc_y[j]),
                gate_width, gate_width,
                theta1=arc_theta1[j], theta2=arc_theta2[j],
                color='green', linestyle='dashed', linewidth=1, clip_on=False, fill=False))



# ->plot drone starting pose area
min_x = max_GatesMinX[-1]; max_x = max_GatesMaxX[-1]
min_y = max_GatesMinY[-1]; max_y = max_GatesMaxY[-1]
ax.add_patch(plt.Rectangle(
    (min_x, min_y),
    max_x - min_x, max_y - min_y,
    color="black", fill=False, linestyle="--"))
plt.arrow((min_x + max_x)/2, (min_y + max_y)/2, np.cos(GatesYaw[-1])*gate_radius/100, np.sin(GatesYaw[-1])*gate_radius/100, width=0.3, head_width=2, color="blue", overhang=.3)
plt.annotate(plot_labels[-1], ((min_x + max_x)/2, (min_y + max_y)/2))


# ->plot minimum, x-/y-axis aligned rectangle enclosing race track
ax.add_patch(plt.Rectangle(
    (-max_RacetrackDimX/2, -max_RacetrackDimY/2),
    max_RacetrackDimX,  max_RacetrackDimY,   
    color="black", fill=False, linestyle="--"))


# -plot obstacle area
for i in range(np.size(det_ObstR)):
    ax.add_patch(plt.Circle((max_ObstX[i], max_ObstY[i]), max_ObstR[i], color='pink', clip_on=True, fill=True))


# ->add quadratic grid
xticks = ax.get_xticks()
yticks = ax.get_yticks()
if (xticks[-1] > yticks[-1]):
    ticks_stepsize = yticks[1] - yticks[0]
    ax.set_xticks(np.arange(xticks[0], xticks[-1], ticks_stepsize))
else:
    ticks_stepsize = xticks[1] - xticks[0]
    ax.set_yticks(np.arange(yticks[0], yticks[-1], ticks_stepsize))
ax.grid()
ax.set_aspect('equal', adjustable='box')
fig.tight_layout()
plt.show()




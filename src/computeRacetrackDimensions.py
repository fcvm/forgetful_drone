import matplotlib.pyplot as plt
from matplotlib.patches import Arc
import numpy as np
from numpy.core.fromnumeric import size
from tensorflow.python.ops.gen_control_flow_ops import switch

RAND_MaxScale = 1.2

#%% Data

# gate dimensions (unity prefab) (ignoring the feet)
RPG_GateWidth = 3.35
RPG_GateMinHeight = 3.7


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
det_RacetrackDimX = det_RacetrackDimX + RPG_GateWidth
det_RacetrackDimY = det_RacetrackDimY + RPG_GateWidth


np.set_printoptions(precision=2)
print("Deterministic Figure-8 Racetrack Gate poses:")
print("\tx:", det_GatesX)
print("\ty:", det_GatesY)
print("\tyaw:", GatesYaw)

print("Minimum, x-/y-axis aligned rectangle enclosing race track [DET.]")
print("\tx:", det_RacetrackDimX)
print("\ty:", det_RacetrackDimY)
print("\tz:", RPG_GateMinHeight)



#%% my randomized racetrack data

# parameters for randomizing
RAND_MaxAxialShift = 0.8
RAND_MinScale = 0.8
RAND_MaxScale = 1.2


# Min sized racetrack
min_GatesMinX = (det_GatesX - RAND_MaxAxialShift) * RAND_MinScale
min_GatesMaxX = (det_GatesX + RAND_MaxAxialShift) * RAND_MinScale
min_RacetrackDimX = max(min_GatesMaxX) - min(min_GatesMinX) + RPG_GateWidth

min_GatesMinY = (det_GatesY - RAND_MaxAxialShift) * RAND_MinScale
min_GatesMaxY = (det_GatesY + RAND_MaxAxialShift) * RAND_MinScale
min_RacetrackDimY = max(min_GatesMaxY) - min(min_GatesMinY) + RPG_GateWidth

print("Minimum, x-/y-axis aligned rectangle enclosing race track [MIN. RAND.]")
print("\tx:", min_RacetrackDimX)
print("\ty:", min_RacetrackDimY)
print("\tz:", (RPG_GateMinHeight+2*RAND_MaxAxialShift)*RAND_MinScale)


# Max sized racetrack
max_GatesMinX = (det_GatesX - RAND_MaxAxialShift) * RAND_MaxScale
max_GatesMaxX = (det_GatesX + RAND_MaxAxialShift) * RAND_MaxScale
max_RacetrackDimX = max(max_GatesMaxX) - min(max_GatesMinX) + RPG_GateWidth

max_GatesMinY = (det_GatesY - RAND_MaxAxialShift) * RAND_MaxScale
max_GatesMaxY = (det_GatesY + RAND_MaxAxialShift) * RAND_MaxScale
max_RacetrackDimY = max(max_GatesMaxY) - min(max_GatesMinY) + RPG_GateWidth

print("Minimum, x-/y-axis aligned rectangle enclosing race track [MIN. RAND.]")
print("\tx:", max_RacetrackDimX)
print("\ty:", max_RacetrackDimY)
print("\tz:", (RPG_GateMinHeight+2*RAND_MaxAxialShift)*RAND_MaxScale)




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
gate_radius = RPG_GateWidth/2
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
                RPG_GateWidth, RPG_GateWidth,
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
                RPG_GateWidth, RPG_GateWidth,
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




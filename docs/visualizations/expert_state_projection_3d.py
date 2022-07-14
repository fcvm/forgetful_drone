import utils
from expert_state_projection_data import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import ArrowStyle

utils.initRCParams()
fig = plt.figure()
ax = fig.add_subplot(projection='3d')

# ss0
ax.scatter(
    ss0['x'], ss0['y'], ss0['z'],
    **ss0['kwargs']
)
# gt0
ax.plot(
    gt0['x'], gt0['y'], gt0['z'],
    **gt0['kwargs']
)

# ss1
ax.scatter(
    ss1['x'], ss1['y'], ss1['z'],
    **ss1['kwargs']
)

# gt1
ax.plot(
    gt1['x'], gt1['y'], gt1['z'],
    **gt1['kwargs']
)

# mds
ax.scatter(
    mds['x'], mds['y'], mds['z'],
    **mds['kwargs']
)

# d
ax.scatter(
    d['x'], d['y'], d['z'],
    **d['kwargs']
)


# a2p, a2d, c
for j in IDXS:

    a2p, a2d, c = a2p_a2d_cl(j)

    ax.arrow3D(
        a2p['x'], a2p['y'], a2p['z'],
        a2p['dx'], a2p['dy'], a2p['dz'],
        mutation_scale=20,
        arrowstyle=ArrowStyle.CurveFilledB(head_length=.4, head_width=.1),
        linewidth=3,
        #linestyle='dashed',
        **a2p['kwargs']
    )
    ax.arrow3D(
        a2d['x'], a2d['y'], a2d['z'],
        a2d['dx'], a2d['dy'], a2d['dz'],
        mutation_scale=20,
        arrowstyle=ArrowStyle.CurveFilledB(head_length=.4, head_width=.1),
        linewidth=3,
        #linestyle='dashed',
        **a2d['kwargs']
    )

    ax.annotate3D(
        c['text'], 
        c['xyz'], 
        xytext=(10, -15), 
        textcoords='offset points'
    )

# ops
ax.scatter(
    ops['x'], ops['y'], ops['z'],
    **ops['kwargs']
)

# ops
ax.scatter(
    nps['x'], nps['y'], nps['z'],
    **nps['kwargs']
)


# Plot configurations
legend = ax.legend(loc='lower right')
ax.set_xlim(XLIM)
ax.set_ylim(YLIM)
ax.set_zlim(ZLIM)
#plt.axis('off')

ax.view_init(44, -99)
ax.dist = 4.7
utils.saveFig(__file__)
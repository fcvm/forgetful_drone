
import utils
from expert_state_projection_data import *
import matplotlib.pyplot as plt




utils.initRCParams()
fig, ax = plt.subplots()


# ss0
ax.scatter(
    ss0['x'], ss0['y'],
    **ss0['kwargs']
)

# gt0
ax.plot(
    gt0['x'], gt0['y'], 
    **gt0['kwargs']
)

# ss1
ax.scatter(
    ss1['x'], ss1['y'],
    **ss1['kwargs']
)

# gt1
ax.plot(
    gt1['x'], gt1['y'], 
    **gt1['kwargs']
)

# mds
ax.scatter(
    mds['x'], mds['y'],
    **mds['kwargs']
)

# d
ax.scatter(
    d['x'], d['y'],
    **d['kwargs']
)


# a2p, a2d, c
for j in IDXS:

    a2p, a2d, c = a2p_a2d_cl(j)
    utils.Arrow(
        ax,
        a2p['x'], a2p['y'],
        a2p['dx'], a2p['dy'],
        a2p['width'],
        **a2p['kwargs']
    )
    utils.Arrow(
        ax,
        a2d['x'], a2d['y'],
        a2d['dx'], a2d['dy'],
        a2d['width'],
        **a2d['kwargs']
    )

    ax.annotate(
        c['text'],
        c['xyz'][:-1],
        (10, -10),
        textcoords=c['textcoords']
    )

# ops
ax.scatter(
    ops['x'], ops['y'],
    **ops['kwargs']
)

# ops
ax.scatter(
    nps['x'], nps['y'],
    **nps['kwargs']
)


# Plot configurations
legend = ax.legend(loc='lower right')
ax.set_xlim(XLIM)
ax.set_ylim(YLIM)
plt.axis('off')
utils.saveFig(__file__)

import numpy as np


ss0_t = np.linspace(0, 1, 8)
ss0_x = np.sin(ss0_t)
ss0_y = ss0_t**1.5
ss0_z = 0.1 * ss0_t
ss0 = {
    't': ss0_t[1:-1],
    'x': ss0_x[1:-1],
    'y': ss0_y[1:-1],
    'z': ss0_z[1:-1],
    'kwargs': {
        'color': 'tab:blue',
        'label': 'Sampled states',
    }
}
gt0 = {
    't': ss0_t,
    'x': ss0_x,
    'y': ss0_y,
    'z': ss0_z,
    'kwargs': {
        'color': 'tab:blue',
        'linestyle': ':',
        'label': 'Global trajectory',
    }
}

ss1_t = ss0_t
ss1_x = ss1_t
ss1_y = - ss1_t**2 + 1
ss1_z = - 0.1 * ss1_t
ss1 = {
    't': ss1_t[1:-1],
    'x': ss1_x[1:-1],
    'y': ss1_y[1:-1],
    'z': ss1_z[1:-1],
    'kwargs': {
        'color': 'tab:blue',
    }
}
gt1 = {
    't': ss1_t,
    'x': ss1_x,
    'y': ss1_y,
    'z': ss1_z,
    'kwargs': {
        'color': 'tab:blue',
        'linestyle': ':',
    }
}


d_x = 0.5
d_y = 0.7
d_z = 0.0
d_p = np.array([d_x, d_y, d_z])
d = {
    'x': d_x,
    'y': d_y,
    'z': d_z,
    'kwargs': {
        'color': 'tab:orange',
        's': 100,
        'label': 'Drone position'
    }
}




md0 = 10**10
for i in range(len(ss0_t)):
    ss0_p = np.array([ss0_x[i], ss0_y[i], ss0_z[i]])
    dist = np.linalg.norm(ss0_p - d_p)
    if dist < md0:
        md0 = dist
        mds_idx = i
md1 = 10**10
for i in range(len(ss1_t)):
    ss1_p = np.array([ss1_x[i], ss1_y[i], ss1_z[i]])
    dist = np.linalg.norm(ss1_p - d_p)
    if dist < md1:
        md1 = dist
        mds_idx = i
assert md0 != md1, 'Min. distance state is on current part of global trajectory!'
mds = {
    't': ss1_t[mds_idx],
    'x': ss1_x[mds_idx],
    'y': ss1_y[mds_idx],
    'z': ss1_z[mds_idx],
    'kwargs': {
        'color': 'tab:blue',
        'marker': 's',
        's': 100,
        'label': 'Min. dist. state'
    }
}


IDXS = [2, 3, 4, 5]
def a2p_a2d_cl (i: int):
    SCALE=10

    ps_p = np.array([ss0_x[i], ss0_y[i], ss0_z[i]])
    as_p = np.array([ss0_x[i - 1], ss0_y[i - 1], ss0_z[i - 1]])

    a2p = ps_p - as_p
    a2p_norm = np.linalg.norm(a2p) * SCALE
    a2p /= a2p_norm

    a2d = (d_p - as_p) / a2p_norm

    c_lval = np.dot(a2p, a2d) * SCALE**2
    c_text = '$\\texttt{\#' + f"{i - IDXS[0]}:}}\ {c_lval:.2f}"
    if i in IDXS[:-1]:
        c_text += '\\nless 1$'
    else:
        c_text += ' < 1$'

    return (
        {
            'x': ss0_x[i - 1],
            'y': ss0_y[i - 1],
            'z': ss0_z[i - 1],
            'dx': a2p[0],
            'dy': a2p[1],
            'dz': a2p[2],
            'width': 0.008,
            'kwargs': {
                'color': 'tab:green',
            }
        },
        {
            'x': ss0_x[i - 1],
            'y': ss0_y[i - 1],
            'z': ss0_z[i - 1],
            'dx': a2d[0],
            'dy': a2d[1],
            'dz': a2d[2],
            'width': 0.008,
            'kwargs': {
                'color': 'tab:orange',
            }
        },
        {
            'text': c_text,
            'xyz': tuple(as_p),
            'textcoords': 'offset points'
        }
    )
ops = {
    'x': ss0_x[IDXS[0]],
    'y': ss0_y[IDXS[0]],
    'z': ss0_z[IDXS[0]],
    'kwargs': {
        'color': 'tab:green',
        's': 100,
        'label': 'Last proj. state'
    }
}
nps = {
    'x': ss0_x[IDXS[-1]],
    'y': ss0_y[IDXS[-1]],
    'z': ss0_z[IDXS[-1]],
    'kwargs': {
        'color': 'tab:green',
        'marker': '*',
        's': 200,
        'label': 'New proj. state'
    }
}


XLIM = [0.1, 0.9] 
YLIM = [0.0, 0.95]
ZLIM = [-0.5, 0.5]
import utils
import matplotlib.pyplot as plt
import numpy as np

### DATA ###
x = np.linspace(-10, 10, 200)
def sig(x):
    return 1/(1 + np.exp(-x))
def sig_d1(x):
    return sig(x) * (1 - sig(x)) 

sigmoid = {
    'x': x,
    'fx': sig(x),
    'kwargs': {
        'color': 'tab:blue',
        'label': '$\\sigma(x)$',
        'linestyle': '-'
    }
}

sigmoid_d1 = {
    'x': x,
    'fx': sig_d1(x),
    'kwargs': {
        'color': 'tab:blue',
        'label': '$\\sigma\'(x)$',
        'linestyle': ':'
    }
}


### PLOT CONFIG ###
legend_loc = 'lower right'
grid_on = True
xlabel = '$x$'
ylabel = '$f(x)$'
xlim = [-4.1, 4.1] 
ylim = [-1.1, 1.1]


### PLOTTING ###
utils.initRCParams()
fig, ax = plt.subplots()

ax.plot(
    sigmoid['x'], sigmoid['fx'], 
    **sigmoid['kwargs']
)
ax.plot(
    sigmoid_d1['x'], sigmoid_d1['fx'], 
    **sigmoid_d1['kwargs']
)


ax.legend(loc=legend_loc)
ax.grid(grid_on)
plt.xlabel(xlabel)
plt.ylabel(ylabel)
ax.set_xlim(xlim)
ax.set_ylim(ylim)
ax.set_aspect('equal',adjustable='box')
utils.saveFig(__file__)
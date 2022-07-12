import utils
import matplotlib.pyplot as plt
import numpy as np

### DATA ###
x = np.linspace(-10, 10, 200)
sigmoid = {
    'x': x,
    'fx': 1/(1 + np.exp(-x)),
    'kwargs': {
        'color': 'tab:blue',
        'label': '$\\sigma(x)$',
        'linestyle': ':'
    }
}
tanh = {
    'x': x,
    'fx': np.tanh(x),
    'kwargs': {
        'color': 'tab:orange',
        'label': '$\\tanh(x)$',
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
    tanh['x'], tanh['fx'], 
    **tanh['kwargs']
)


ax.legend(loc=legend_loc)
ax.grid(grid_on)
plt.xlabel(xlabel)
plt.ylabel(ylabel)
ax.set_xlim(xlim)
ax.set_ylim(ylim)
ax.set_aspect('equal',adjustable='box')
utils.saveFig(__file__)
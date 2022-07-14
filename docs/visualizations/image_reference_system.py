from turtle import color
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import pathlib
import plot_utils
import numpy as np




### Plot configuration
plt.rcParams['text.usetex'] = True
plt.rcParams['text.latex.preamble'] = [
    r'\usepackage{amssymb}',
    r'\usepackage{amsmath}',
    r'\usepackage{mathtools}'
]

fig, ax = plt.subplots()


img = mpimg.imread(pathlib.Path(__file__).parent.resolve()/'gfx/camera_frame_00400.jpg')
ax.imshow(img, alpha=1)

H, W, C = img.shape

HEADLENGTH=50


xticks = [
    (0, '$-1$'), 
    (W/4, ''), 
    (W/2, '$x=0$'), 
    (3/4*W, '$0.5$'), 
    (W, '$1$')
]
yticks = [
    (0, '$1$'), 
    (H/4, '$0.5$'), 
    (H/2, '$y=0$'), 
    (3/4*H, '$-0.5$'), 
    (H, '$-1$')
]



for tick in xticks:
    ax.plot(
        (tick[0], tick[0]),
        (0, H),
        color='tab:blue'
    )
    ax.annotate(
        tick[1],
        (tick[0], H),
        (0, -15),
        ha='center',
        textcoords='offset points',
        fontsize=12,
        color='tab:blue'
    )
for tick in yticks:
    ax.plot(
        (0, W),
        (tick[0], tick[0]),
        color='tab:blue'
    )
    ax.annotate(
        tick[1],
        (W, tick[0]),
        (15, -3),
        ha='center',
        textcoords='offset points',
        fontsize=12,
        color='tab:blue'
    )



ccolor = 'tab:orange'

X = - 0.428
Y = 0.136
x = int(X * W/2 + W/2)
y = int(-Y * H/2 + H/2)

ax.scatter(
    x, y,
    color=ccolor,
    marker='+',
    s=200,
    linewidth=2
)

ax.plot(
    (x, x),
    (y, H),
    color=ccolor,
    linewidth=2,
    linestyle=':'
)
ax.annotate(
        f"${X}$",
        (x, H),
        (0, -15),
        ha='center',
        textcoords='offset points',
        fontsize=12,
        color=ccolor
    )

ax.plot(
    (x, W),
    (y, y),
    color=ccolor,
    linewidth=2,
    linestyle=':'
)
ax.annotate(
        f"${Y}$",
        (W, y),
        (15, -3),
        ha='center',
        textcoords='offset points',
        fontsize=12,
        color=ccolor
    )


#for ytick in yticks:
#    ax.plot(
#        (0, W),
#        (ytick, ytick),
#        color='tab:blue'
#    )


#plot_utils.annotatedVector(
#    ax=ax,
#    xy1=tuple(origin),
#    xy2=tuple(origin+ex),
#    text='${}_\\textbf{I} \\underline e_x^\\text{I}$',
#    textdxdy=(-20, -30),
#    width=0.01,
#    color='tab:orange',
#    alpha=1
#)
#plot_utils.annotatedVector(
#    ax=ax,
#    xy1=tuple(origin),
#    xy2=tuple(origin+ey),
#    text='${}_\\textbf{I} \\underline e_y^\\text{I}$',
#    textdxdy=(30, -30),
#    width=0.01,
#    color='tab:orange',
#    alpha=1
#)

#plot_utils.annotatedVector(
#    ax=ax,
#    xy1=(0, H/2),
#    xy2=(W+HEADLENGTH, H/2),
#    text='$x$',
#    textdxdy=(-20, -30),
#    width=0.01,
#    color='tab:orange',
#    alpha=1
#)

#plot_utils.annotatedVector(
#    ax=ax,
#    xy1=tuple(0, H/2),
#    xy2=tuple(W+40, H/2),
#    text='$y$',
#    textdxdy=(30, -30),
#    width=0.01,
#    color='tab:orange',
#    alpha=1
#)





#mpl.rcParams['legend.fontsize'] = 10
#ax.legend(loc='lower right')

ax.set_aspect('equal',adjustable='box')

#ax.set_xticks([])
#ax.set_yticks([])
plt.axis('off')
ax.set_xlim([0, W+HEADLENGTH])
#ax.set_ylim([-0.1, 3.6])

plt.savefig(
    pathlib.Path(__file__).parent.resolve()/f"{pathlib.Path(__file__).stem}.pdf", 
    bbox_inches='tight',
    #bbox_inches=ax.get_window_extent().transformed(fig.dpi_scale_trans.inverted()),
    pad_inches=0
)
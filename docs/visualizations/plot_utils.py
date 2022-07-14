from turtle import width
from matplotlib.lines import Line2D
from matplotlib.patches import Arc, FancyArrow
import math
import numpy as np

from typing import Tuple


from matplotlib.axes._axes import Axes


def deg__from__rad(angle: float):
    return angle / np.pi * 180.0
def rad__from__deg(angle: float):
    return angle / 180.0 * np.pi


def Angle2D(
    l1: Line2D,
    l2: Line2D,
    radius: float,
    **kwargs
):
    l1_x1 = l1.get_xdata(orig=False)[0]
    l1_x2 = l1.get_xdata(orig=False)[-1]
    l1_y1 = l1.get_ydata(orig=False)[0]
    l1_y2 = l1.get_ydata(orig=False)[-1]
    
    l2_x1 = l2.get_xdata(orig=False)[0]
    l2_x2 = l2.get_xdata(orig=False)[-1]
    l2_y1 = l2.get_ydata(orig=False)[0]
    l2_y2 = l2.get_ydata(orig=False)[-1]
    
    if not l1_x2 - l1_x1 == 0:
        am = (l1_y2 - l1_y1) / (l1_x2 - l1_x1)
        a_vertical = False
    else:
        a_vertical = True

    if not l2_x2 - l2_x1 == 0:
        bm = (l2_y2 - l2_y1) / (l2_x2 - l2_x1)
        b_vertical = False
    else:
        b_vertical = True

    if (not a_vertical) and (not b_vertical):
        cx0 = (am * l1_x1 - bm * l2_x1 - l1_y1 + l2_y1) / (am - bm)
        cy0 = am * (cx0 - l1_x1) + l1_y1
    elif a_vertical:
        cx0 = l1_x1
        cy0 = bm * (cx0 - l2_x1) + l2_y1
    elif b_vertical:
        cx0 = l2_x1
        cy0 = am * (cx0 - l1_x1) + l1_y1

    thetaA=math.atan2(l1_y2 - l1_y1, l1_x2 - l1_x1)
    thetaB=math.atan2(l2_y2 - l2_y1, l2_x2 - l2_x1)
    
    return Arc(
        xy=(cx0, cy0),
        width=2*radius, height=2*radius,
        angle=0,
        theta1=deg__from__rad(min(thetaA, thetaB)),
        theta2=deg__from__rad(max(thetaA, thetaB)),
        **kwargs
    )



def Angle2DAnnotation(
    ax,
    Angle2D: Arc,
    text: str,
    radius: float,
    textshift,
    color,
):
    theta = rad__from__deg((Angle2D.theta1 + Angle2D.theta2) / 2)
    dx = radius * np.cos(theta)
    dy = radius * np.sin(theta)
    ax.annotate(
        text, 
        (Angle2D._center[0] + dx, Angle2D._center[1] + dy),
        textshift,
        textcoords='offset points', 
        c=color
    )

def Angle2DArrow(
    direction,
    ax,
    Angle2D: Arc,
    radius: float,
    **kwargs
):
    x = 0.999
    
    if direction:
        theta1 = rad__from__deg(Angle2D.theta2)
        theta2 = x * theta1 + (1 - x) * rad__from__deg(Angle2D.theta1)
    else:
        theta1 = rad__from__deg(Angle2D.theta1)
        theta2 = x * theta1 + (1 - x) * rad__from__deg(Angle2D.theta2)
    
    x1 = radius * np.cos(theta1) + Angle2D._center[0]
    y1 = radius * np.sin(theta1) + Angle2D._center[1]
    x2 = radius * np.cos(theta2) + Angle2D._center[0]
    y2 = radius * np.sin(theta2) + Angle2D._center[1]
    ax.arrow(
        x1, y1, x1 - x2, y1 - y2,
        **kwargs
    )








def annotatedVector(
    ax: Axes,
    xy1: Tuple[float, float],
    xy2: Tuple[float, float],
    text: str,
    textdxdy: Tuple[float, float],
    width: float,
    color: str,
    alpha: float
):
    x = xy1[0]
    y = xy1[1]
    dx = xy2[0] - x
    dy = xy2[1] - y

    
    length = (dx**2 + dy**2)**0.5
    print(ax.bbox.width)
    headlength = 5 * width * ax.bbox.width / 60
    if length < headlength:
        x = xy2[0] - dx / length * headlength
        y = xy2[1] - dy / length * headlength
        dx = xy2[0] - x
        dy = xy2[1] - y

    ax.quiver(
        x, y, dx, dy, 
        angles='xy', 
        scale_units='xy',
        scale=1,
        width=width,
        minshaft=0,
        minlength=0,
        color=color,
        alpha=alpha
    )
    ax.annotate(
        text,
        xy2,
        textdxdy,
        textcoords='offset points', 
        color=color,
        alpha=alpha
    )

def annotatedAngle(
    ax: Axes,
    aux1xy1xy2: Tuple[Tuple[float, float], Tuple[float, float]],
    aux2xy1xy2: Tuple[Tuple[float, float], Tuple[float, float]],
    radius: float,
    text: str,
    textdxdy: Tuple[float, float],
    width: float,
    color: str,
    alpha: float
):
    aux1x1 = aux1xy1xy2[0][0]
    aux1y1 = aux1xy1xy2[0][1]
    aux1x2 = aux1xy1xy2[1][0]
    aux1y2 = aux1xy1xy2[1][1]
    
    print(aux1x1, aux1y1, aux1x2, aux1y2)

    aux2x1 = aux2xy1xy2[0][0]
    aux2y1 = aux2xy1xy2[0][1]
    aux2x2 = aux2xy1xy2[1][0]
    aux2y2 = aux2xy1xy2[1][1]

    print(aux2x1, aux2y1, aux2x2, aux2y2)
    
    # arc
    if not aux1x2 - aux1x1 == 0:
        aux1m = (aux1y2 - aux1y1) / (aux1x2 - aux1x1)
        aux1vertical = False
    else:
        aux1vertical = True
    if not aux2x2 - aux2x1 == 0:
        aux2m = (aux2y2 - aux2y1) / (aux2x2 - aux2x1)
        aux2vertical = False
    else:
        aux2vertical = True
    if (not aux1vertical) and (not aux2vertical):
        arcx = (aux1m * aux1x1 - aux2m * aux2x1 - aux1y1 + aux2y1) / (aux1m - aux2m)
        arcy = aux1m * (arcx - aux1x1) + aux1y1
    elif aux1vertical and (not aux2vertical):
        arcx = aux1x1
        arcy = aux2m * (arcx - aux2x1) + aux2y1
    elif (not aux1vertical) and aux2vertical:
        arcx = aux2x1
        arcy = aux1m * (arcx - aux1x1) + aux1y1
    aux1a=math.atan2(aux1y2 - aux1y1, aux1x2 - aux1x1)
    aux2a=math.atan2(aux2y2 - aux2y1, aux2x2 - aux2x1)

    # annotated arrow
    x = 0.999
    arra2 = aux2a
    arra1 = (1 - x) * aux1a + x * aux2a
    arrx1 = radius * np.cos(arra1) + arcx
    arry1 = radius * np.sin(arra1) + arcy
    arrx2 = radius * np.cos(arra2) + arcx
    arry2 = radius * np.sin(arra2) + arcy

    
    
    
    
    
    ax.add_patch(Arc(
        xy=(arcx, arcy),
        width=2*radius, height=2*radius,
        angle=0,
        theta1=deg__from__rad(min(aux1a, aux2a)),
        theta2=deg__from__rad(max(aux1a, aux2a)),
        color=color,
        alpha=alpha
    ))

    

    
    annotatedVector(
        ax=ax,
        xy1=(arrx1, arry1),
        xy2=(arrx2, arry2),
        text=text,
        textdxdy=textdxdy,
        width=width,
        color=color,
        alpha=alpha
    )




class AnnotatedArrow(FancyArrow):
    """
    Like FancyArrow, but annotated.
    """

    def __str__(self):
        return "AnnotatedArrow()"

    def __init__(
        self,
        ax,
        text,
        text_offset_xy,
        color,
        x, 
        y, 
        dx, 
        dy,
        width=0.001, 
        length_includes_head=True,
        head_width=None, 
        head_length=None, 
        shape='full', 
        overhang=0,
        head_starts_at_zero=False, 
        **kwargs
    ) -> None:
        super().__init__(
            x, 
            y, 
            dx, 
            dy,
            width, 
            length_includes_head,
            head_width, 
            head_length, 
            shape, 
            overhang,
            head_starts_at_zero,
            fc=color,
            ec=color,
            **kwargs
        )
        ax.add_patch(self)
        ax.annotate(
            text,
            (x + dx, y + dy),
            text_offset_xy,
            textcoords='offset points', 
            color=color
        )





class AnnotatedAngleArrow(Arc):
    """
    Annotated arc, between two Line2D instances, 
    with arrow from first to second line.
    """

    def __str__(self):
        return "AnnotatedAngleArrow()"

    def __init__(
        self,
        ax,
        text,
        text_offset_xy,
        color,
        l1: Line2D,
        l2: Line2D,
        radius: float,
        width=0.001, 
        length_includes_head=False,
        head_width=None, 
        head_length=None, 
        shape='full', 
        overhang=0,
        head_starts_at_zero=False,
        **kwargs
    ) -> None:
        l1_x1 = l1.get_xdata(orig=False)[0]
        l1_x2 = l1.get_xdata(orig=False)[-1]
        l1_y1 = l1.get_ydata(orig=False)[0]
        l1_y2 = l1.get_ydata(orig=False)[-1]
        
        l2_x1 = l2.get_xdata(orig=False)[0]
        l2_x2 = l2.get_xdata(orig=False)[-1]
        l2_y1 = l2.get_ydata(orig=False)[0]
        l2_y2 = l2.get_ydata(orig=False)[-1]
        
        if not l1_x2 - l1_x1 == 0:
            am = (l1_y2 - l1_y1) / (l1_x2 - l1_x1)
            a_vertical = False
        else:
            a_vertical = True

        if not l2_x2 - l2_x1 == 0:
            bm = (l2_y2 - l2_y1) / (l2_x2 - l2_x1)
            b_vertical = False
        else:
            b_vertical = True

        if (not a_vertical) and (not b_vertical):
            x0 = (am * l1_x1 - bm * l2_x1 - l1_y1 + l2_y1) / (am - bm)
            y0 = am * (x0 - l1_x1) + l1_y1
        elif a_vertical:
            x0 = l1_x1
            y0 = bm * (x0 - l2_x1) + l2_y1
        elif b_vertical:
            x0 = l2_x1
            y0 = am * (x0 - l1_x1) + l1_y1

        l1_theta=math.atan2(l1_y2 - l1_y1, l1_x2 - l1_x1)
        l2_theta=math.atan2(l2_y2 - l2_y1, l2_x2 - l2_x1)

        # Arc        
        super().__init__(
            xy=(x0, y0),
            width=2*radius, height=2*radius,
            angle=0,
            theta1=deg__from__rad(min(l1_theta, l2_theta)),
            theta2=deg__from__rad(max(l1_theta, l2_theta)),
            color=color,
            **kwargs
        )
        ax.add_patch(self)

        # Annotation
        lc_theta = (l1_theta + l2_theta) / 2
        lc_dx = radius * np.cos(lc_theta)
        lc_dy = radius * np.sin(lc_theta)
        ax.annotate(
            text, 
            (x0 + lc_dx, y0 + lc_dy),
            text_offset_xy,
            textcoords='offset points', 
            c=color
        )

        # Arrow
        x = 0.999
        l2_theta_2 = l2_theta
        l2_theta_1 = (1 - x) * l1_theta + x * l2_theta
        x1 = radius * np.cos(l2_theta_1) + x0
        y1 = radius * np.sin(l2_theta_1) + y0
        x2 = radius * np.cos(l2_theta_2) + x0
        y2 = radius * np.sin(l2_theta_2) + y0
        ax.arrow(
            x1, y1, x2 - x1, y2 - y1,
            color=color
            **kwargs
        )
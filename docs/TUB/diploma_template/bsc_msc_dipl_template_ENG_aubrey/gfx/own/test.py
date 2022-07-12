import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

#input values
x=[1,10,50,100,150]
y=[1,300,350,250,50]
z=[0,1]
def edgecoord(pointx,pointy,pointz):
    edgex=[pointx[0],pointx[1],pointx[1],pointx[0]]
    edgey=[pointy[0],pointy[1],pointy[1],pointy[0]]
    edgez=[pointz[0],pointz[0],pointz[1],pointz[1]]
    return list(zip(edgex,edgey,edgez))

def coordConvert(x,y,lheight,uheight):
    if len(x) != len(y) and len(x)>2:
        return
    vertices=[]
    #Top layer
    vertices.append(list(zip(x,y,list(np.full(len(x),uheight)))))
    # Side layers
    for it in np.arange(len(x)):
        it1=it+1
        if it1>=len(x):
            it1=0
        vertices.append(edgecoord([x[it],x[it1]],[y[it],y[it1]],[lheight,uheight]))
    #Bottom layer
    vertices.append(list(zip(x,y,list(np.full(len(x),lheight)))))
    print(np.array(vertices))
    return vertices

vec=coordConvert(x,y,z[0],z[1])

plt.figure()
plt.subplot(111,projection='3d')
plt.gca().add_collection3d(Poly3DCollection(vec, alpha=.75,edgecolor='k', facecolor='teal'))
plt.xlim([0,200])
plt.ylim([0,400])
plt.show()
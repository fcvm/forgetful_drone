import matplotlib.pyplot as plt
import numpy as np
from typing import List, Tuple

import utils
import math

precolor = 'tab:blue'
postcolor = 'tab:orange'

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

    @staticmethod 
    def slerp (q1: np.ndarray, q2: np.ndarray, t: float) -> np.ndarray:
        cos_half_theta = np.inner (q1, q2)
        if abs(cos_half_theta) >= 1.0: 
            return q1

        half_theta = math.acos (cos_half_theta)
        sin_half_theta = np.sqrt (1.0 - cos_half_theta ** 2)

        if (abs (sin_half_theta) < 0.001):
            return Quat.fromWXYZ (
                (q1 [0] + q2 [0]) / 2,
                (q1 [1] + q2 [1]) / 2,
                (q1 [2] + q2 [2]) / 2,
                (q1 [3] + q2 [3]) / 2,
            )
        
        ratio1 = np.sin ((1 - t) * half_theta) / sin_half_theta
        ratio2 = np.sin (t * half_theta) / sin_half_theta

        return Quat.fromWXYZ (
            q1 [0] * ratio1 + q2 [0] * ratio2,
            q1 [1] * ratio1 + q2 [1] * ratio2,
            q1 [2] * ratio1 + q2 [2] * ratio2,
            q1 [3] * ratio1 + q2 [3] * ratio2,
        )
    
   
class Pose:
    def __init__ (self, p : np.ndarray = None, q : np.ndarray = None) -> None:
        if p is None : p = Pos.fromXYZ (0.0, 0.0, 0.0)
        if q is None : q = Quat.fromWXYZ (0.0, 0.0, 0.0, 0.0)
        Pos.assertPos (p)
        Quat.assertQuat (q)
        self.p = p
        self.q = q



class Figure8:
    def __init__ (self, gate_poses : List [Pose] = None) -> None:
        self.gateWidth = 3.35
        self.gateMinHeight = 3.7
        self.gateConstYawTwist = - np.pi / 2
        self.frontGateIdx = -2
        
        self.randMaxScale = 1.2                         - 0.4
        self.randMinScale = 0.8                         
        self.randMaxAxShift = 0.8                       * 2
        self.randMaxYawTwist = 5.0 / 180 * np.pi        * 3

        if gate_poses is not None:
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
            x = gate_poses [i].p [0]
            y = gate_poses [i].p [1]
            z = gate_poses [i].p [2]

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
        
        xmin -= self.gateWidth * 0.6 
        ymin -= self.gateWidth * 0.6 
        zmin -= self.gateWidth * 0.6 
        xmax += self.gateWidth * 0.6 
        ymax += self.gateWidth * 0.6 
        zmax += self.gateWidth * 0.6 
        
        return ([xmin, xmax], [ymin, ymax], [zmin, zmax])

    def plotTrackXY (self, 
        ax,
        gate_poses : List [Pose],
        drone_pose : Pose,
        color : str,
        alpha : float,
        annotate : bool
    ) -> None:
        
        # gates
        for i in range (len (gate_poses)):
            x = gate_poses [i].p [0]
            y = gate_poses [i].p [1]
            yaw = Quat.yaw (gate_poses [i].q)
            lbl = i

            dx = np.cos (yaw) * self.gateWidth / 2 
            dy = np.sin (yaw) * self.gateWidth / 2 
            ax.arrow (x - dx, y - dy, 2 * dx, 2 * dy, 
                width=0.1, head_width=0.0, color=color,
                alpha=alpha
            )

            #ax.add_patch(
            #    plt.Circle ((x, y), 0.3, color='black', clip_on=False, fill=True)
            #)
            if annotate:
                ax.annotate (f"${lbl}$", (x - 3, y - 3), fontsize=20)

        # drone
        dx = np.cos (Quat.yaw (drone_pose.q)) * self.gateWidth / 200
        dy = np.sin (Quat.yaw (drone_pose.q)) * self.gateWidth / 200
        ax.arrow (
            drone_pose.p [0], 
            drone_pose.p [1], 
            dx,
            dy, 
            width=0.3, head_width=1.0, color=color, #overhang=.3
            alpha=alpha
        )
        #if annotate:
        #    ax.annotate ('Init. drone', (drone_pose.p [0], drone_pose.p [1]))
        

    def randTrack (self, gate_poses : List [Pose]) -> List [Pose]:
        shift_poses = []
        for pose in gate_poses:
            shift = self.randMaxAxShift * np.random.uniform (-1.0, 1.0, (3,))
            shift [2] += self.randMaxAxShift
            p = Pose ()
            p.p = pose.p + shift
            p.q = pose.q
            shift_poses.append (p)
        
        scale_poses = []
        scale = np.random.uniform (self.randMinScale, self.randMaxScale)
        for pose in shift_poses: 
            p = Pose ()
            p.p = pose.p * scale
            p.q = pose.q
            scale_poses.append (p)

        twist_poses = []
        for pose in scale_poses:
            twist = self.randMaxYawTwist * np.random.uniform (-1.0, 1.0)
            p = Pose ()
            p.p = pose.p
            p.q = Quat.multiply (pose.q, Quat.fromAngAx (twist, UnitVecZ))
            twist_poses.append (p)
        
        redir_poses = self.redirTrack (twist_poses)

        utils.initRCParams ()
        xlim, ylim, _ = self.getTrackSize (
            gate_poses + shift_poses+ scale_poses + twist_poses
        )
        alpha_pre = 0.4
        
        fig, ax = plt.subplots ()
        

        self.plotTrackXY (ax, 
            gate_poses, self.compDronePose (gate_poses),
            precolor, alpha_pre, False
        )
        self.plotTrackXY (ax, 
            shift_poses, self.compDronePose (shift_poses),
            postcolor, 1.0, True
        )
        ax.set_aspect ('equal', adjustable='box')
        ax.set_xlim (xlim)
        ax.set_ylim (ylim)
        #legend = ax.legend(loc='lower right')
        plt.axis('off')
        fig.tight_layout()
        utils.saveFig(__file__, '_fig8_shift', transparent=True)

        fig, ax = plt.subplots ()
        self.plotTrackXY (ax, 
            shift_poses, self.compDronePose (shift_poses),
            precolor, alpha_pre, False
        )
        self.plotTrackXY (ax, 
            scale_poses, self.compDronePose (scale_poses),
            postcolor, 1.0, True
        )
        ax.set_aspect ('equal', adjustable='box')
        ax.set_xlim (xlim)
        ax.set_ylim (ylim)
        #legend = ax.legend(loc='lower right')
        plt.axis('off')
        fig.tight_layout()
        utils.saveFig(__file__, '_fig8_scale', transparent=True)

        fig, ax = plt.subplots ()
        self.plotTrackXY (ax, 
            scale_poses, self.compDronePose (scale_poses),
            precolor, alpha_pre, False
        )
        twist_drone = self.compDronePose (twist_poses)
        twist_drone.q = Quat.multiply (twist_drone.q, Quat.fromAngAx (np.pi, UnitVecZ) )
        self.plotTrackXY (ax, 
            twist_poses, twist_drone,
            postcolor, 1.0, True
        )
        ax.set_aspect ('equal', adjustable='box')
        ax.set_xlim (xlim)
        ax.set_ylim (ylim)
        #legend = ax.legend(loc='lower right')
        plt.axis('off')
        fig.tight_layout()
        utils.saveFig(__file__, '_fig8_twist', transparent=True)

        fig, ax = plt.subplots ()
        self.plotTrackXY (ax, 
            twist_poses, twist_drone,
            postcolor, alpha_pre, False
        )
        self.plotTrackXY (ax, 
            redir_poses, self.compDronePose (redir_poses),
            postcolor, 1.0, True
        )
        ax.set_aspect ('equal', adjustable='box')
        ax.set_xlim (xlim)
        ax.set_ylim (ylim)
        #legend = ax.legend(loc='lower right')
        plt.axis('off')
        fig.tight_layout()
        utils.saveFig(__file__, '_fig8_redir', transparent=True)
        
        return twist_poses
    

    def redirTrack (self, gate_poses : List [Pose]) -> List [Pose]:
        N = len (gate_poses)

        redir_poses = []
        for i in range (N):
            p = gate_poses [(2 * N + self.frontGateIdx -i) % N]
            p.q = Quat.multiply (p.q, Quat.fromAngAx (np.pi, UnitVecZ))
            redir_poses.append (p)

        return redir_poses



class Gap (Figure8):

    def __init__ (self, 
        gate_poses_narrow : List [Pose],
        gate_poses_wide : List [Pose]
    ) -> None:
        Figure8.__init__ (self)

        self.gatePosesDetN = gate_poses_narrow
        self.dronePoseDetN = self.compDronePose (self.gatePosesDetN)

        self.gatePosesDetW = gate_poses_wide
        self.dronePoseDetW = self.compDronePose (self.gatePosesDetW)

        self.gatePosesRand = self.randTrack (
            self.gatePosesDetN, self.gatePosesDetW
        )
        self.dronePoseRand = self.compDronePose (self.gatePosesRand)

    def randAvg (self,
        gate_poses_n : List [Pose],
        gate_poses_w : List [Pose]
    ) -> List [Pose]:
        weight = np.random.uniform (0.0, 1.0)
        
        avg_poses = []
        for i in range (len (gate_poses_n)):
            pn = gate_poses_n [i]
            pw = gate_poses_w [i]

            ap = Pose ()
            ap.p = pn.p + (pw.p - pn.p) * weight
            ap.q = Quat.slerp (pn.q, pw.q, weight)
            avg_poses.append (ap)

        return avg_poses

    def randTrack (self, 
        gate_poses_n : List [Pose],
        gate_poses_w : List [Pose]
    ) -> List [Pose]:
        avg_poses = self.randAvg (gate_poses_n, gate_poses_w)


        shift_poses = []
        for pose in avg_poses:
            shift = self.randMaxAxShift * np.random.uniform (-1.0, 1.0, (3,))
            shift [2] += self.randMaxAxShift
            p = Pose ()
            p.p = pose.p + shift
            p.q = pose.q
            shift_poses.append (p)
        
        scale_poses = []
        scale = np.random.uniform (self.randMinScale, self.randMaxScale)
        for pose in shift_poses: 
            p = Pose ()
            p.p = pose.p * scale
            p.q = pose.q
            scale_poses.append (p)

        twist_poses = []
        for pose in scale_poses:
            twist = self.randMaxYawTwist * np.random.uniform (-1.0, 1.0)
            p = Pose ()
            p.p = pose.p
            p.q = Quat.multiply (pose.q, Quat.fromAngAx (twist, UnitVecZ))
            twist_poses.append (p)
        
        redir_poses = self.redirTrack (twist_poses)

        utils.initRCParams ()
        plt.rcParams['legend.fontsize'] = 20
        xlim, ylim, _ = fig8.getTrackSize (
            gate_poses_n + gate_poses_w + shift_poses+ scale_poses + twist_poses
        )
        alpha_pre = 0.4
        
        fig, ax = plt.subplots ()
        self.plotTrackXY (ax, 
            avg_poses, self.compDronePose (avg_poses),
            postcolor, 1.0, True
        )
        for pn, pw in zip (gate_poses_n, gate_poses_w):
            x = [pn.p [0], pw.p [0]]
            y = [pn.p [1], pw.p [1]]
            ax.plot (x, y, color=precolor)#, alpha=alpha_pre)
        ax.set_aspect ('equal', adjustable='box')
        ax.set_xlim (xlim)
        ax.set_ylim (ylim)
        #legend = ax.legend(loc='lower right')
        plt.axis('off')
        fig.tight_layout()
        utils.saveFig(__file__, '_gap_avg', transparent=True)

        fig, ax = plt.subplots ()
        self.plotTrackXY (ax, 
            avg_poses, self.compDronePose (avg_poses),
            precolor, alpha_pre, False
        )
        self.plotTrackXY (ax, 
            shift_poses, self.compDronePose (shift_poses),
            postcolor, 1.0, True
        )
        ax.set_aspect ('equal', adjustable='box')
        ax.set_xlim (xlim)
        ax.set_ylim (ylim)
        #legend = ax.legend(loc='lower right')
        plt.axis('off')
        fig.tight_layout()
        utils.saveFig(__file__, '_gap_shift', transparent=True)

        fig, ax = plt.subplots ()
        self.plotTrackXY (ax, 
            shift_poses, self.compDronePose (shift_poses),
            precolor, alpha_pre, False
        )
        self.plotTrackXY (ax, 
            scale_poses, self.compDronePose (scale_poses),
            postcolor, 1.0, True
        )
        ax.set_aspect ('equal', adjustable='box')
        ax.set_xlim (xlim)
        ax.set_ylim (ylim)
        #legend = ax.legend(loc='lower right')
        plt.axis('off')
        fig.tight_layout()
        utils.saveFig(__file__, '_gap_scale')

        fig, ax = plt.subplots ()
        self.plotTrackXY (ax, 
            scale_poses, self.compDronePose (scale_poses),
            precolor, alpha_pre, False
        )
        twist_drone = self.compDronePose (twist_poses)
        twist_drone.q = Quat.multiply (twist_drone.q, Quat.fromAngAx (np.pi, UnitVecZ) )
        self.plotTrackXY (ax, 
            twist_poses, twist_drone,
            postcolor, 1.0, True
        )
        ax.set_aspect ('equal', adjustable='box')
        ax.set_xlim (xlim)
        ax.set_ylim (ylim)
        #legend = ax.legend(loc='lower right')
        plt.axis('off')
        fig.tight_layout()
        utils.saveFig(__file__, '_gap_twist', transparent=True)

        fig, ax = plt.subplots ()
        self.plotTrackXY (ax, 
            twist_poses, twist_drone,
            precolor, alpha_pre, False
        )
        self.plotTrackXY (ax, 
            redir_poses, self.compDronePose (redir_poses),
            postcolor, 1.0, True
        )
        ax.set_aspect ('equal', adjustable='box')
        ax.set_xlim (xlim)
        ax.set_ylim (ylim)
        #legend = ax.legend(loc='lower right')
        plt.axis('off')
        fig.tight_layout()
        utils.saveFig(__file__, '_gap_redir', transparent=True)
        
        return twist_poses

    







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

GapDetX = [-20.45, -12.55, -4.15, 4.85, 16.95, 16.95, 5.45, -4.95, -12.95, -21.05, -24.25]
GapDetYN = [-8.65, -11.15, -9.35, -4.95, -2.25, 2.25, 4.45, 7.95, 9.65, 6.65, -1.0]
GapDetYW = [-8.65, -11.15, -9.35, -5.95, -5.25, 5.25, 5.45, 7.95, 9.65, 6.65, -1.0]
GapDetZ = [2.0] * len (GapDetX)
GapDetYaw = [1.13, -1.57, -1., -1.4, 1.57, 1.57, 1.4, 1.2, -1.53, -0.77, 0.07]
GapDetNPoses = []
for x, y, z, yaw in zip (GapDetX, GapDetYN, GapDetZ, GapDetYaw):
    GapDetNPoses.append (Pose (
        Pos.fromXYZ (x, y, z),
        Quat.fromAngAx (yaw, UnitVecZ)
    ))
GapDetWPoses = []
for x, y, z, yaw in zip (GapDetX, GapDetYW, GapDetZ, GapDetYaw):
    GapDetWPoses.append (Pose (
        Pos.fromXYZ (x, y, z),
        Quat.fromAngAx (yaw, UnitVecZ)
    ))


fig8 = Figure8 (Fig8DetPoses)
gap = Gap (GapDetNPoses, GapDetWPoses)


"""三种 computeSegmentQuats 改法的对比。"""
import numpy as np
from hand_rig import HandRig, axis_angle_deg, quat_angle_deg, mat2quat, FINGERS, MCP_POS
from seg_quat import gt_segment_frames, bone_frame, N_SEG, SEG_NAMES

_o = HandRig.joint_angles
HandRig.joint_angles = lambda self,t,motion='grasp': {f:[(fl*0.35,ab) for fl,ab in _o(self,t,motion).items().__iter__().__next__()[1]] for f in []} or {f:[(fl*0.35,ab) for fl,ab in v] for f,v in _o(self,t,motion).items()}

def frame_ref_global(d, prev=None, wristR=None):
    return bone_frame(d)

def frame_ref_dorsal(d, prev=None, wristR=None):
    """参考向量 = 手背法线（腕部R的z轴），投影正交化。连续且有解剖意义。"""
    n = np.linalg.norm(d)
    if n < 1e-12: return np.eye(3)
    x = d/n
    ref = wristR[:,2]
    y = np.cross(ref, x); ny = np.linalg.norm(y)
    if ny < 0.15:                      # 骨轴几乎平行于手背法线，退化
        ref = wristR[:,0]
        y = np.cross(ref, x); ny = np.linalg.norm(y)
        if ny < 1e-9: return np.eye(3)
    y /= ny
    return np.column_stack([x, y, np.cross(x, y)])

def dirs_current(P, anchors):
    D = np.zeros((N_SEG,3))
    for f in range(5):
        b = 5+f*3
        for k in range(3):
            m = b+k
            a_ = m if k==0 else m-1
            b_ = m+1 if k==0 else m
            D[1+f*3+k] = P[b_]-P[a_]
    return D

def dirs_fixed(P, anchors):
    """prox 用 MCP锚点->pp，mid 用 pp->mp，dist 用 mp->dp。三段互不相同。"""
    D = np.zeros((N_SEG,3))
    for f in range(5):
        b = 5+f*3
        D[1+f*3+0] = P[b+0]-anchors[f]
        D[1+f*3+1] = P[b+1]-P[b+0]
        D[1+f*3+2] = P[b+2]-P[b+1]
    return D

def run(dirfn, framefn, noise=0.5, frames=1500, seed=1):
    rng = np.random.default_rng(seed)
    rig = HandRig('right')
    ax=[[] for _ in range(N_SEG)]; jp=[[] for _ in range(N_SEG)]; prevQ=None
    for i in range(frames):
        t=i/120.0
        P,Qgt,Qp,B = rig.forward(t)
        Rp,Tp = rig.palm_pose(t)
        anchors = np.array([Rp@MCP_POS[f]+Tp for f in FINGERS])
        Pn = P + rng.normal(0,noise,P.shape)
        Dgt = gt_segment_frames(Qgt,B)
        D = dirfn(Pn, anchors)
        Q=np.zeros((N_SEG,4))
        for s in range(1,N_SEG):
            Q[s]=mat2quat(framefn(D[s], wristR=Rp))
            ax[s].append(axis_angle_deg(D[s],Dgt[s]))
        if prevQ is not None:
            for s in range(1,N_SEG): jp[s].append(quat_angle_deg(Q[s],prevQ[s]))
        prevQ=Q.copy()
    A=[x for s in range(1,N_SEG) for x in ax[s]]
    J=[x for s in range(1,N_SEG) for x in jp[s]]
    return np.percentile(A,50),np.percentile(A,90),np.max(A),np.percentile(J,99),np.max(J)

rows=[("现状 (代码原样)",           dirs_current, frame_ref_global),
      ("仅修 ref 向量 (改法①)",     dirs_current, frame_ref_dorsal),
      ("仅修索引配对 (改法②)",       dirs_fixed,   frame_ref_global),
      ("①+② 一起",                  dirs_fixed,   frame_ref_dorsal)]
print(f"{'方案':26s}{'骨轴中位':>9s}{'p90':>8s}{'max':>8s}{'跳变p99':>10s}{'跳变max':>10s}")
for n,df,ff in rows:
    r=run(df,ff)
    print(f"{n:26s}{r[0]:8.1f}°{r[1]:7.1f}°{r[2]:7.1f}°{r[3]:9.1f}°{r[4]:9.1f}°")

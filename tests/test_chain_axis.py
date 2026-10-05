"""链式续解直接产出骨轴 vs 现状(从预测点做差分)。
真值用 rig 的 seg_R 第0列(段自身的骨轴)，不是从真值点差分 —— 那才是"段朝向"的定义。
"""
import sys, numpy as np, onnxruntime as ort
sys.path.insert(0,'/home/claude/ai/ai')
from hand_rig import sample_subjects, forward_kinematics, rest_pose_markers
from synth import template_feature
from occlusion import sample_cameras, visibility
import sim_jitter as S

MODEL="/mnt/user-data/uploads/1786716425597_hm20_v6_sk10.onnx"

def rot(v, n, a):
    c,s=np.cos(a),np.sin(a)
    return v*c + np.cross(n,v)*s + n*np.dot(n,v)*(1-c)

def run(n_cam=3, noise=0.9, seed=7, n_frames=240):
    rng=np.random.default_rng(seed)
    sub=sample_subjects(rng,1,left_hand_p=1.0)
    rest,_,_,_=rest_pose_markers(sub)
    tmpl=template_feature(rest,sub.sign).astype(np.float32)
    tv=np.zeros((1,20),np.float32); tv[0,:5]=1
    ang=S.smooth_sequence(rng,n_frames); T=len(ang)
    big=type(sub)(**{k:np.repeat(getattr(sub,k),T,axis=0) for k in sub.__dataclass_fields__})
    mk,segR,segO,joints=forward_kinematics(big,ang)
    anchor=np.asarray(sub.anchor)[0]                     # (5,3) 腕部系
    cams=np.repeat(sample_cameras(rng,1,n_cam=n_cam),T,axis=0)
    vis,_=visibility(mk.astype(np.float32),segO.astype(np.float32),
                     segR.astype(np.float32),joints.astype(np.float32),cams)
    vis=vis.astype(bool)
    sess=ort.InferenceSession(MODEL,providers=["CPUExecutionProvider"])
    prev=mk[0].astype(np.float32).copy(); pmask=np.ones((1,20),np.float32)

    st={f:dict(plane=None,pip=None,u0=None,dipk=0.66,pm=None) for f in range(5)}
    errs={'现状':[], '链式直出':[]}
    errsA={'现状(差分)':[], '链式(情形A)':[]}
    for t in range(T):
        seen=vis[t]; pts=mk[t][seen]+rng.normal(0,noise,(seen.sum(),3))
        if len(pts)<3: continue
        out=sess.run(None,{"points":pts[None].astype(np.float32),
                           "mask":np.ones((1,len(pts)),np.float32),"tmpl":tmpl,
                           "tmpl_valid":tv,"prev":prev[None].astype(np.float32),"prev_mask":pmask})
        pos=out[1][0]; prev=pos.copy()
        for f in range(5):
            pp,mp,dp=5+3*f,6+3*f,7+3*f
            gt=segR[t,1+3*f+1,:,0]                        # 中节真骨轴
            gt=gt/np.linalg.norm(gt)
            # ---- 现状：从(可能是预测的)点做差分 pp->dp ----
            v=pos[dp]-pos[pp]; nv=np.linalg.norm(v)
            if nv>1e-6 and not (seen[pp] and seen[dp]):
                errs['现状'].append(np.degrees(np.arccos(np.clip(np.dot(v/nv,gt),-1,1))))
            # ---- 学习(三点全可见) ----
            if seen[pp] and seen[mp] and seen[dp]:
                u0=pos[pp]-anchor[f]; u0/=np.linalg.norm(u0)
                u1=pos[mp]-pos[pp]; L=np.linalg.norm(u1); u1/=L
                u2=pos[dp]-pos[mp]; u2/=np.linalg.norm(u2)
                nr=np.cross(u0,u1); nn=np.linalg.norm(nr)
                if nn>1e-3:
                    nr/=nn
                    if st[f]['plane'] is not None and np.dot(nr,st[f]['plane'])<0: nr=-nr
                    st[f]['plane']=nr if st[f]['plane'] is None else 0.9*st[f]['plane']+0.1*nr
                    st[f]['plane']/=np.linalg.norm(st[f]['plane'])
                pipA=np.arccos(np.clip(np.dot(u0,u1),-1,1))
                st[f]['pip']=pipA; st[f]['pm']=L; st[f]['u0']=u0
                dipA=np.arccos(np.clip(np.dot(u1,u2),-1,1))
                if pipA>0.35: st[f]['dipk']=0.95*st[f]['dipk']+0.05*np.clip(dipA/pipA,0,1.2)
                continue
            # ---- 情形A：pp,mp 可见、dp 遮挡。比【远节】骨轴 ----
            if seen[pp] and seen[mp] and not seen[dp] and st[f]['plane'] is not None:
                gd = segR[t,1+3*f+2,:,0]; gd = gd/np.linalg.norm(gd)
                u0=pos[pp]-anchor[f]; u0/=np.linalg.norm(u0)
                u1=pos[mp]-pos[pp]; u1/=np.linalg.norm(u1)
                pipA=np.arccos(np.clip(np.dot(u0,u1),-1,1))
                u2c=rot(u1,st[f]['plane'],st[f]['dipk']*pipA)
                errsA['链式(情形A)'].append(np.degrees(np.arccos(np.clip(np.dot(u2c,gd),-1,1))))
                vd=pos[dp]-pos[mp]; nd=np.linalg.norm(vd)
                if nd>1e-6:
                    errsA['现状(差分)'].append(np.degrees(np.arccos(np.clip(np.dot(vd/nd,gd),-1,1))))
            if st[f]['plane'] is None or st[f]['pip'] is None: continue
            if False:
                u0=pos[pp]-anchor[f]; u0/=np.linalg.norm(u0)
                if st[f]['u0'] is not None:
                    d=np.arccos(np.clip(np.dot(u0,st[f]['u0']),-1,1))
                    ax=np.cross(st[f]['u0'],u0)
                    if np.dot(ax,st[f]['plane'])<0: d=-d
                    st[f]['pip']=float(np.clip(st[f]['pip']+np.clip(d,-0.14,0.14),0,1.92))
                st[f]['u0']=u0
            else:
                u0=st[f]['u0']
                if u0 is None: continue
            u1=rot(u0,st[f]['plane'],st[f]['pip'])
            errs['链式直出'].append(np.degrees(np.arccos(np.clip(np.dot(u1,gt),-1,1))))
    print(f"{'方案':<16}{'n':>6}{'中位':>8}{'p90':>8}{'max':>8}")
    for k,v in errsA.items():
        if not v: print(f'{k:<16}     0   (无样本)'); continue
        v=np.array(v)
        print(f'{k:<16}{len(v):>6}{np.median(v):>8.1f}{np.percentile(v,90):>8.1f}{v.max():>8.1f}')
    for k,v in errs.items():
        if not v: print(f'{k:<16}     0   (无样本)'); continue
        v=np.array(v)
        print(f'{k:<16}{len(v):>6}{np.median(v):>8.1f}{np.percentile(v,90):>8.1f}{v.max():>8.1f}')

if __name__=="__main__":
    for nc in (3,4):
        print(f"--- {nc}相机 ---"); run(n_cam=nc)

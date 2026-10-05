# -*- coding: utf-8 -*-
"""rom_offline_dump.py —— 拿【已有的 v5/v6 录制】离线跑 ROM 标定

    python3 tools/pcrec/rom_offline_dump.py 录制.pcrec /tmp/a.obs 1.0
    g++ -std=c++17 -O1 -I src tools/pcrec/rom_offline_run.cpp -o /tmp/romrun
    /tmp/romrun gate /tmp/a.obs                  # 单段
    /tmp/romrun gate /tmp/a.obs /tmp/b.obs ...   # 多段当补标累加

【为什么需要它】v7 之前的录制没有 JointSolve/RomCalib 块，但 JointOut 里有
qSolve、Skeleton 里有 20 点位置和分段四元数 —— 逐维状态和 curl 都能从这两样
重建出来。于是【不用重录】就能回答"这段素材到底能不能标定成功"。

重建出来的逐维状态是近似的（v5 没记 anchorsValid，退化只能按
acos(dot(a0,a1)) < 阈值 判），所以结论要当【下界】看：真机上有 v7 的
proxAxisSrc 之后只会更准，不会更差。

本脚本同时对齐新版 solveJointAngles：
· 四指 PIP 取绝对值，统一输出“弯曲量”；
· 输出 OBS2 格式，每帧带 3 组方向参考（本指 curl / 四指共同 curl /
  拇CMC屈专用参考），供 rom_offline_run 复现新版多候选判方向逻辑。
"""
import sys, struct
sys.path.insert(0,'tools/pcrec'); import pcrec as P
import numpy as np

def col0(q):
    w,x,y,z=q[...,0],q[...,1],q[...,2],q[...,3]
    return np.stack([1-2*(y*y+z*z),2*(x*y+z*w),2*(x*z-y*w)],-1)

def build(path, degen_gate_deg=1.0):
    r=P.load(path)
    jo=r.jointout
    sk=np.array([s for _,s in r.skeleton],dtype=P.SKEL)
    n=min(len(jo),len(sk))
    jo=jo[:n]; sk=sk[:n]
    pos=sk["pos"].reshape(-1,20,3); sq=sk["segQuat"].reshape(-1,16,4); ss=sk["segSource"]
    Q=jo["qSolve"].astype(np.float64)
    # 与新版 solveJointAngles 对齐：四指 PIP 统一输出“弯曲量”，恒为非负。
    # 旧录制里 PIP 的 hingeSigned 会翻号，直接拿原始 qSolve 会复现小指反转。
    Q[:, [6,9,12,15]] = np.abs(Q[:, [6,9,12,15]])
    ST=np.full((n,16),4,np.uint8)
    CURL=np.zeros((n,5)); CV=np.zeros((n,5),np.uint8)
    for f in range(5):
        sP=1+3*f; sM=sP+1; sD=sP+2
        pp=5+3*f; mp=pp+1; dp=pp+2
        a0=col0(sq[:,sP,:]); v=pos[:,mp]-pos[:,pp]
        a1=v/np.maximum(np.linalg.norm(v,axis=-1,keepdims=True),1e-9)
        ang=np.degrees(np.arccos(np.clip(np.sum(a0*a1,-1),-1,1)))
        degen = ang < degen_gate_deg
        lPM=np.linalg.norm(pos[:,mp]-pos[:,pp],axis=-1)
        lMD=np.linalg.norm(pos[:,dp]-pos[:,mp],axis=-1)
        lPD=np.linalg.norm(pos[:,dp]-pos[:,pp],axis=-1)
        den=lPM+lMD
        ok=(den>1e-3)&(lPM>1e-3)&(lMD>1e-3)
        A=pos[:,mp]-pos[:,pp]; B=pos[:,dp]-pos[:,mp]
        cf=np.sum(A*B,-1)/np.maximum(np.linalg.norm(A,axis=-1)*np.linalg.norm(B,axis=-1),1e-9)
        fold=np.arccos(np.clip(cf,-1,1))
        CURL[:,f]=np.clip(fold/1.6,0,1); CV[:,f]=ok
        meas=np.isin(ss[:,sP],(2,3)) & np.isin(ss[:,sM],(2,3))
        qual=np.where(meas,4,3).astype(np.uint8)
        base = 0 if f==0 else 4+(f-1)*3
        pipIdx = 2 if f==0 else base+2
        mcpIdx = [0,1] if f==0 else [base,base+1]
        fv=jo["fingerValid"][:,f].astype(bool); mv=jo["mcpValid"].astype(bool)
        ST[:,pipIdx]=np.where(~fv,1,np.where(degen,2,qual))
        for k in mcpIdx: ST[:,k]=np.where(~fv,1,np.where(~mv,1,qual))
        if f==0:
            dOk=np.isin(ss[:,sD],(2,3))
            ST[:,3]=np.where(~fv,1,np.where(dOk,qual,1))
    # ---- 多候选方向参考。约定全部是“越大=越握拳”。----
    # 候选0：本指 curl。
    # 候选1：四指共同 curl（四指一起握拳/张开，抗单指噪声）。
    # 候选2：拇CMC屈专用参考（拇指尖->食指MCP 的负距离；越近=越屈曲）。
    SIGNREF=np.zeros((n,3,16),np.float64)
    SIGNREF_VALID=np.zeros((n,3,16),np.uint8)
    for i in P.FLEX_IDX:
        f=0 if i<4 else 1+(i-4)//3
        SIGNREF[:,0,i]=CURL[:,f]
        SIGNREF_VALID[:,0,i]=CV[:,f]
    valid4=CV[:,1:].astype(bool)
    with np.errstate(invalid="ignore"):
        common=np.nanmean(np.where(valid4,CURL[:,1:],np.nan),axis=1)
    common_n=valid4.sum(axis=1)
    for i in P.FLEX_IDX:
        m=(common_n>=2)&np.isfinite(common)
        SIGNREF[m,1,i]=common[m]
        SIGNREF_VALID[m,1,i]=1
    thumb_len=np.linalg.norm(pos[:,7]-pos[:,5],axis=1)
    d78=np.linalg.norm(pos[:,7]-pos[:,8],axis=1)
    ok=thumb_len>1e-3
    SIGNREF[ok,2,0]=-d78[ok]/thumb_len[ok]
    SIGNREF_VALID[ok,2,0]=1

    ts=jo["frameTsNs"].astype(np.int64)
    return n,Q,ST,CURL,CV,SIGNREF,SIGNREF_VALID,ts

if __name__=="__main__":
    src,out,gate=sys.argv[1],sys.argv[2],float(sys.argv[3])
    start=int(sys.argv[4]) if len(sys.argv)>4 else 0
    end=int(sys.argv[5]) if len(sys.argv)>5 else -1
    n,Q,ST,CURL,CV,SIGNREF,SIGNREF_VALID,ts=build(src,gate)
    if end<0 or end>n: end=n
    start=max(0,min(start,end)); end=max(start,end)
    Q=Q[start:end]; ST=ST[start:end]; CURL=CURL[start:end]; CV=CV[start:end]
    SIGNREF=SIGNREF[start:end]; SIGNREF_VALID=SIGNREF_VALID[start:end]; ts=ts[start:end]
    n=end-start
    with open(out,"wb") as f:
        f.write(b"OBS2")
        f.write(struct.pack("<i",n))
        for i in range(n):
            f.write(Q[i].astype("<f8").tobytes())
            f.write(ST[i].tobytes())
            f.write(CURL[i].astype("<f8").tobytes())
            f.write(CV[i].tobytes())
            f.write(SIGNREF[i].astype("<f8").tobytes())
            f.write(SIGNREF_VALID[i].tobytes())
            f.write(struct.pack("<q",int(ts[i])))
    print(f"{out}  {n} 帧（原文件 {start}..{end}）")

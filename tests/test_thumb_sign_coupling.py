import pcrec, numpy as np
def load(F):
    r=pcrec.load(F); S=r.skeleton
    pos=np.array([s["pos"].reshape(20,3) for _,s in S]); obs=np.array([s["observed"] for _,s in S]).astype(bool)
    vis=obs[:,7]; mcp=np.full(len(S),np.nan); ip=np.full(len(S),np.nan)
    for i in range(len(S)):
        if not vis[i]: continue
        P,M,D=pos[i,5],pos[i,6],pos[i,7]
        u0=P-pos[i,:5].mean(0); u0/=np.linalg.norm(u0)
        u1=(M-P)/np.linalg.norm(M-P); u2=(D-M)/np.linalg.norm(D-M)
        mcp[i]=np.arccos(np.clip(u0@u1,-1,1)); ip[i]=np.arccos(np.clip(u1@u2,-1,1))
    # 遮挡期 MCP 仍可测 —— 用几何补全（m5,m6 全程可见）
    for i in range(len(S)):
        if np.isnan(mcp[i]) and obs[i,5] and obs[i,6]:
            P,M=pos[i,5],pos[i,6]
            u0=P-pos[i,:5].mean(0); u0/=np.linalg.norm(u0)
            u1=(M-P)/np.linalg.norm(M-P)
            mcp[i]=np.arccos(np.clip(u0@u1,-1,1))
    runs=[];st=None
    for i in range(len(S)):
        if vis[i] and st is None: st=i
        if not vis[i] and st is not None: runs.append((st,i-1)); st=None
    if st is not None: runs.append((st,len(S)-1))
    occ=[];st=None
    for i in range(len(S)):
        if not vis[i] and st is None: st=i
        if vis[i] and st is not None: occ.append((st,i-1)); st=None
    return mcp,ip,[x for x in runs if x[1]-x[0]>=10],occ,len(S)

GATE=np.deg2rad(0.5); K=2.35
def track(ip0,mcp_series,t0,t,k=K,gate=GATE):
    """从 t0（最后一次可见）走到 t0+t，每帧按 MCP 的增量走"""
    x=ip0
    for j in range(t0+1,t0+t+1):
        dM=mcp_series[j]-mcp_series[j-1]
        if np.isnan(dM) or abs(dM)<gate: continue
        x+=k*dM
        x=np.clip(x,np.deg2rad(-15),np.deg2rad(85))
    return x

FS=[("第1段","/mnt/user-data/uploads/1787415781097_pcrec_遮挡压力_20260823_002157.pcrec"),
    ("第2段","/mnt/user-data/uploads/pcrec_刚体运动_20260823_052130.pcrec")]
print("=== 合成遮挡（可见段内，样本多）===")
print(f"{'录制':<6}{'N帧':>5}{'样本':>7}{'保持':>9}{'跟随MCP k=2.35':>16}{'k=1.0':>9}{'k=4.0':>9}")
for nm,F in FS:
    mcp,ip,runs,occ,N=load(F)
    for Nf in (5,10,20):
        H=[];T=[];T1=[];T4=[]
        for a,b in runs:
            for s0 in range(a,b-Nf):
                if np.isnan(ip[s0]) or np.isnan(ip[s0+Nf]): continue
                tr=ip[s0+Nf]
                H.append(abs(tr-ip[s0]))
                T.append(abs(tr-track(ip[s0],mcp,s0,Nf)))
                T1.append(abs(tr-track(ip[s0],mcp,s0,Nf,k=1.0)))
                T4.append(abs(tr-track(ip[s0],mcp,s0,Nf,k=4.0)))
        if len(H)<15: continue
        f=lambda v:np.degrees(np.median(v))
        print(f"{nm:<6}{Nf:5d}{len(H):7d}{f(H):9.1f}{f(T):16.1f}{f(T1):9.1f}{f(T4):9.1f}")
print("\n=== 四段真实遮挡（第2段），预测出口值 ===")
mcp,ip,runs,occ,N=load(FS[1][1])
print(f"{'时长':>5}{'入口':>7}{'真值出口':>9}{'保持':>8}{'跟随k=2.35':>12}{'k=1.0':>8}{'k=4.0':>8}")
E={'保持':[],'k2.35':[],'k1':[],'k4':[]}
for a,b in occ:
    if a<2 or b+1>=N or np.isnan(ip[a-1]) or np.isnan(ip[b+1]): continue
    i0=ip[a-1]; tr=ip[b+1]; D_=b-a+1
    p={'保持':i0,'k2.35':track(i0,mcp,a-1,D_+1),
       'k1':track(i0,mcp,a-1,D_+1,k=1.0),'k4':track(i0,mcp,a-1,D_+1,k=4.0)}
    print(f"{D_:5d}{np.degrees(i0):7.1f}{np.degrees(tr):9.1f}" +
          "".join(f"{np.degrees(abs(tr-p[k])):{w}.1f}" for k,w in
                  [('保持',8),('k2.35',12),('k1',8),('k4',8)]))
    for k in E: E[k].append(abs(tr-p[k]))
print(f"\n{'模型':<10}{'中位':>8}{'最大':>8}")
for k,v in sorted(E.items(),key=lambda kv:np.median(kv[1])):
    print(f"{k:<10}{np.degrees(np.median(v)):8.1f}{np.degrees(max(v)):8.1f}")

import numpy as np
FRAME=8.333            # 120fps
PHASE=[0.0,1.4,2.9,5.8]  # 四台相机相位（真值 = 5.8ms）
def old(win,trials=3000):
    sp=[]
    for _ in range(trials):
        # 批次从"第一台到达"起算：找一个随机时刻之后各相机的首次投递
        t=np.random.rand()*FRAME*10
        first=[ph+np.ceil((t-ph)/FRAME)*FRAME for ph in PHASE]
        t0=min(first)                       # 批次起点
        inbatch=[f-t0 for f in first if f-t0 < win]   # 只有窗口内到的才进 hash
        if len(inbatch)>=2: sp.append(max(inbatch)-min(inbatch))
    return (np.percentile(sp,95), len(sp)/trials) if sp else (0,0)
def new(win,trials=3000):
    sp=[]
    for _ in range(trials):
        now=np.random.rand()*FRAME*10+100
        last=[now-((now-ph)%FRAME) for ph in PHASE]   # 各相机最近一次投递
        sp.append(max(last)-min(last))
    return np.percentile(sp,95), 1.0
print(f"真实相位差 = {max(PHASE)-min(PHASE):.1f} ms   帧周期 = {FRAME:.1f} ms\n")
print(f"{'窗口ms':>7}{'旧法p95':>10}{'旧法建议':>10}{'能测到的帧占比':>14}{'新法p95':>10}{'新法建议':>10}")
for w in (3,5,7,9,12,20,40):
    o,r=old(w); n,_=new(w)
    print(f"{w:7d}{o:10.2f}{int(np.ceil(o*1.5))+1:10d}{r*100:13.0f}%{n:10.2f}{int(np.ceil(n*1.5))+1:10d}")
print("\n旧法：窗口<帧周期时相位靠后的相机进不来 -> 低估 -> 调大窗口测出的值反而变大")
print("新法：跟窗口无关，恒定")

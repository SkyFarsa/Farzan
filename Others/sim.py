import numpy as np, math, csv, json
rng=np.random.default_rng(42)
R=0.06;B=0.48;TPR=203;DT=0.05
def run(path,tpr_err=0,b_err=0,rl_err=0,rr_err=0,dropout=0,glitch=0):
    # true robot vs estimator. path: list of (v,w,T)
    x=y=th=0.0; ex=ey=eth=0.0; ll=lr=0.0; last=(0,0); out=[]
    Bt=B+b_err; tpr_est=TPR; tpr_true=TPR*(1+tpr_err)
    Rl=R*(1+rl_err); Rr=R*(1+rr_err)
    for v,w,T in path:
        for _ in range(int(T/DT)):
            vl=v-w*Bt/2; vr=v+w*Bt/2
            dl=vl*DT; dr=vr*DT
            ds=(dl+dr)/2; dth=(dr-dl)/Bt
            x+=ds*math.cos(th+dth/2); y+=ds*math.sin(th+dth/2); th+=dth
            ll+=dl/(2*math.pi*Rl)*tpr_true; lr+=dr/(2*math.pi*Rr)*tpr_true
            if rng.random()<dropout: out.append((x,y,ex,ey)); continue
            L=int(ll); Rt=int(lr)
            if glitch and rng.random()<glitch: L=int(str(abs(L))[1:] or 0)  # drop leading digit
            a=(L-last[0])/tpr_est*2*math.pi*R; b=(Rt-last[1])/tpr_est*2*math.pi*R; last=(L,Rt)
            s=(a+b)/2; d=(b-a)/B
            ex+=s*math.cos(eth+d/2); ey+=s*math.sin(eth+d/2); eth+=d
            out.append((x,y,ex,ey))
    return out,(x,y,th),(ex,ey,eth)
def square(side=2.0,v=0.25,w=0.6):
    p=[]
    for _ in range(4): p+= [(v,0,side/v),(0,w,(math.pi/2)/w)]
    return p
def err(tp,ep):
    return math.hypot(tp[0]-ep[0],tp[1]-ep[1]), math.degrees(abs((tp[2]-ep[2]+math.pi)%(2*math.pi)-math.pi))
res={}
# A: ideal square (quantization only)
o,t,e=run(square()); res['A']=err(t,e)
# B: monte carlo
fe=[];he=[]
for _ in range(300):
    o_,t,e=run(square(),tpr_err=rng.normal(0,.005),b_err=rng.normal(0,.008),rl_err=rng.normal(0,.004),rr_err=rng.normal(0,.004))
    a,b=err(t,e); fe.append(a);he.append(b)
res['B']=dict(pos_mean=np.mean(fe),pos_p95=np.percentile(fe,95),hd_mean=np.mean(he),hd_p95=np.percentile(he,95))
# C: sensitivity of 360deg spin to wheel base error (estimator uses 0.48)
sens=[]
for be in np.arange(-0.03,0.0301,0.005):
    o_,t,e=run([(0,0.8,2*math.pi/0.8)],b_err=be); sens.append((round(be*100,1),math.degrees(e[2])-360*0+0 if False else math.degrees(e[2])))
res['C']=sens
# D: calibration 1 rev vs 10 rev
cal={}
for N in (1,5,10):
    est=[TPR*(1+rng.normal(0,0.01/math.sqrt(1)) / N) for _ in range(2000)]
    est=[(TPR*N*(1+rng.normal(0,0.01)) )/N for _ in range(2000)] if False else [ (TPR*N+rng.normal(0,0.01*TPR)+rng.uniform(-1,1))/N for _ in range(2000)]
    cal[N]=float(np.std(est)/TPR*100)
res['D']=cal
# E: packet dropout and glitches
res['E']={}
for name,kw in [('none',{}),('5% dropped',dict(dropout=.05)),('20% dropped',dict(dropout=.2)),('0.1% garbled digits',dict(glitch=.001)),('0.5% garbled digits',dict(glitch=.005))]:
    v=[err(*run(square(),**kw)[1:])  for _ in range(40)]
    res['E'][name]=(float(np.mean([a for a,_ in v])),float(np.mean([b for _,b in v])))
# example drift path
o,t,e=run(square(),tpr_err=.008,b_err=.01,rl_err=.004,rr_err=-.004)
res['path']=[(p[0],p[1],p[2],p[3]) for p in o[::4]]
# CSV test data
with open('sim_test_data.csv','w',newline='') as f:
    w=csv.writer(f); w.writerow(['t_s','true_x','true_y','odom_x','odom_y','pos_err_m'])
    for i,p in enumerate(o[::2]): w.writerow([round(i*2*DT,2),round(p[0],4),round(p[1],4),round(p[2],4),round(p[3],4),round(math.hypot(p[0]-p[2],p[1]-p[3]),4)])
json.dump(res,open('/tmp/res.json','w'),default=float)
print(json.dumps({k:v for k,v in res.items() if k!='path'},indent=1,default=float))

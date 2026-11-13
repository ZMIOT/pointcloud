"""Read-only numerical inspection of a user-selected local XYZ neighbourhood."""
import argparse
from pathlib import Path
import numpy as np
from InspectHull import read_xyz

def voxel(points):
    _,index=np.unique(np.floor(points),axis=0,return_inverse=True)
    count=np.bincount(index)
    return np.column_stack([np.bincount(index,weights=points[:,j])/count for j in range(3)])

ap=argparse.ArgumentParser();ap.add_argument('raw',type=Path);ap.add_argument('body',type=Path);ap.add_argument('center',nargs=3,type=float)
args=ap.parse_args();center=np.array(args.center);raw=read_xyz(args.raw);body=read_xyz(args.body)
raw=voxel(raw[np.linalg.norm(raw-center,axis=1)<18])-center
body=voxel(body[np.linalg.norm(body-center,axis=1)<18])-center
rng=np.random.default_rng(123);best=(-1,None,0)
for i in range(512):
    a,b,c=raw[rng.choice(len(raw),3,replace=False)];n=np.cross(b-a,c-a);length=np.linalg.norm(n)
    if length<1e-6:continue
    n/=length
    if abs(n[1])>.7:continue
    offset=n@a;score=np.sum(abs(raw@n-offset)<=.6)
    if score>best[0]:best=(score,n,offset)
score,n,offset=best;t=np.cross(n,[1,0,0]);t/=np.linalg.norm(t);v=np.cross(n,t)
p=raw[abs(raw@n-offset)<5.4];x=p@t/18;y=p@v/18;z=p@n
b=np.column_stack([np.ones(len(p)),x,y,x*x,x*y,y*y]);coef=np.array([offset,0,0,0,0,0])
for i in range(4):
    weights=np.minimum(1,.6/np.maximum(abs(z-b@coef),1e-9));coef=np.linalg.solve(b.T@(weights[:,None]*b)+np.eye(6)*1e-7*len(p),b.T@(weights*z))
res=abs(z-b@coef);x=body@t/18;y=body@v/18;z=body@n
bd=np.column_stack([np.ones(len(body)),x,y,x*x,x*y,y*y]);contact=abs(z-bd@coef)<2.8
print('raw/body',len(raw),len(body),'normal',n,'plane offset',offset,'plane support',score)
print('coefficients',coef,'quad inlier fraction',np.mean(res<=.6),'contacts',np.sum(contact))
print('contact uv bounds',np.min(np.column_stack([x,y])[contact],axis=0),np.max(np.column_stack([x,y])[contact],axis=0))
print('raw residual quantiles',np.quantile(res,[0,.25,.5,.75,.9,1]))

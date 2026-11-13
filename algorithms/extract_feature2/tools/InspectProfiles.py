from pathlib import Path
import sys
import numpy as np
from PIL import Image, ImageDraw
from InspectSeams import records
d=records(Path(sys.argv[1]));ids=d['frame_index'][:,0]
canvas=Image.new('RGB',(1500,900),(12,18,26));draw=ImageDraw.Draw(canvas)
for k,f in enumerate([1400,1600,1800,2200,2800,3050]):
    xyz=np.column_stack([d[a][np.abs(ids-f)<=1,0] for a in ('x','y','z')]);c=xyz.mean(0)
    _,_,vectors=np.linalg.svd(xyz-c,full_matrices=False);n=vectors[-1];n*=1 if n[0]>0 else -1
    u=np.array([0.,1.,0.]);u-=n*np.dot(n,u);u/=np.linalg.norm(u);v=np.cross(n,u)
    for row,uv in enumerate([xyz[:,[1,2]]-c[[1,2]],np.column_stack([(xyz-c)@u,(xyz-c)@v])]):
        x0=(k%3)*500;y0=(k//3)*450+row*210;lo=uv.min(0);hi=uv.max(0);scale=min(450/(hi[0]-lo[0]),160/max(1,hi[1]-lo[1]))
        ij=np.rint((uv-lo)*[scale,-scale]+[x0+20,y0+190]).astype(int)
        for x,y in ij:draw.point((x,y),fill=(40,190,240))
        draw.text((x0+15,y0+10),f'Frame {f} '+('YZ' if row==0 else 'LOCAL')+f' normal {np.round(n,2)}',fill='white')
canvas.save(sys.argv[2])

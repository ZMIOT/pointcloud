"""Same-coordinate projections of raw input and measured-point crop versions."""
import argparse
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw
from InspectHull import read_xyz

ap=argparse.ArgumentParser()
ap.add_argument('input',type=Path);ap.add_argument('before',type=Path);ap.add_argument('after',type=Path);ap.add_argument('output',type=Path)
args=ap.parse_args()
clouds=[read_xyz(p) for p in (args.input,args.before,args.after)]
recovered_path=Path(str(args.after)+'.recovered.pcd')
recovered=read_xyz(recovered_path) if recovered_path.exists() else np.empty((0,3))
w,h=900,560
for view,axes in [('xz',[0,2]),('xy',[0,1]),('yz',[1,2])]:
    lo=clouds[1][:,axes].min(0)-4;hi=clouds[1][:,axes].max(0)+4
    out=Image.new('RGB',(3*w,h),(12,18,26))
    for col,(label,xyz) in enumerate(zip(('Raw input','1.6 body','Protected measured points'),clouds)):
        xy=np.rint((xyz[:,axes]-lo)/(hi-lo)*[w-40,h-70]+[20,20]).astype(int)
        valid=np.all((xy>=0)&(xy<[w,h-35]),axis=1);xy=xy[valid]
        density=np.zeros((h,w),np.int32);np.add.at(density,(h-1-xy[:,1],xy[:,0]),1)
        value=np.minimum(1,np.log1p(density)/4)
        rgb=np.empty((h,w,3),np.uint8)
        for c,(base,color) in enumerate(zip((12,18,26),(40,192,245))):rgb[:,:,c]=base+(color-base)*value
        if col==2 and len(recovered):
            xy=np.rint((recovered[:,axes]-lo)/(hi-lo)*[w-40,h-70]+[20,20]).astype(int)
            valid=np.all((xy>=0)&(xy<[w,h-35]),axis=1);xy=xy[valid]
            rgb[h-1-xy[:,1],xy[:,0]]=[70,255,90]
        out.paste(Image.fromarray(rgb),(col*w,0));ImageDraw.Draw(out).text((col*w+20,12),label,fill='white')
    out.save(args.output.with_name(args.output.stem+'-'+view+'.png'))

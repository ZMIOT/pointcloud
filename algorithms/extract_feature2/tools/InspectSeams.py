"""Orthographic evidence plots; preserve actual segment boundaries (no gap bridges)."""
import argparse
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw
from InspectHull import read_xyz

def records(path):
    with path.open('rb') as f:
        h = {}
        while True:
            line = f.readline().decode('ascii').split()
            if line and not line[0].startswith('#'): h[line[0]] = line[1:]
            if line and line[0] == 'DATA': break
        assert h['DATA'] == ['binary']
        fields = [(n if n != '_' else f'pad{i}', '<'+{'F':'f','I':'i','U':'u'}[t]+s, (int(c),))
                  for i,(n,t,s,c) in enumerate(zip(h['FIELDS'],h['TYPE'],h['SIZE'],h['COUNT']))]
        return np.frombuffer(f.read(), np.dtype(fields))

if __name__ == '__main__':
    ap=argparse.ArgumentParser();ap.add_argument('directory',type=Path);args=ap.parse_args()
    body=read_xyz(args.directory/'hull_body.pcd')
    seams=[records(args.directory/(s+'_seam.pcd')) for s in ('left','right')]
    lo,hi=body.min(0),body.max(0)
    for axes,name in [([0,1],'xy'),([0,2],'xz'),([1,2],'yz')]:
        w,h=1500,650
        def pixels(xyz):
            scale=min((w-80)/(hi[axes[0]]-lo[axes[0]]),(h-80)/(hi[axes[1]]-lo[axes[1]]))
            uv=(xyz[:,axes]-lo[axes])*scale
            return np.rint(uv*np.array([1,-1])+[40,h-40]).astype(int)
        ij=pixels(body);rgb=np.zeros((h,w,3),np.uint8);rgb[:]=(12,18,26);rgb[ij[:,1],ij[:,0]]=(32,92,122)
        image=Image.fromarray(rgb);draw=ImageDraw.Draw(image)
        draw.text((25,15),f'2.txt protected hull - {name.upper()} | left yellow, right green | gaps NOT connected',fill='white')
        for side,color in zip(seams,[(255,200,40),(80,255,125)]):
            xyz=np.column_stack([side[a][:,0] for a in ('x','y','z')]);uv=pixels(xyz)
            ids=side['segment_id'][:,0]
            for sid in np.unique(ids):
                points=[tuple(x) for x in uv[ids==sid]]
                if len(points)>1:draw.line(points,fill=color,width=3)
                for x,y in (points[0],points[-1]):draw.ellipse((x-3,y-3,x+3,y+3),outline=color)
        image.save(args.directory/f'seams-{name}.png')
    for side in seams:
        xyz=np.column_stack([side[a][:,0] for a in ('x','y','z')]);ids=side['segment_id'][:,0]
        print('points',len(side),'bounds',xyz.min(0),xyz.max(0),'segments',len(np.unique(ids)))
        for sid in np.unique(ids):
            mask=ids==sid;sub=xyz[mask];q=side['quality_flags'][mask,0]
            frames=side['frame_index'][mask,0] if 'frame_index' in side.dtype.names else np.arange(len(sub))
            print(' segment',sid,'n',len(sub),'frames',frames[0],frames[-1],'x',sub[:,0].min(),sub[:,0].max(),'interpolated',np.count_nonzero(q&1),'weak',np.count_nonzero(q&6),'max step',np.linalg.norm(np.diff(sub,axis=0),axis=1).max(initial=0))

"""Render raw XYZ projections and optional crop results without a GUI."""
import argparse
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw

def read_xyz(path):
    if path.suffix.lower() == '.txt':
        return np.loadtxt(path, skiprows=1, usecols=(1, 2, 3))
    with path.open('rb') as f:
        header = {}
        while True:
            line = f.readline().decode('ascii').strip().split()
            if line and not line[0].startswith('#'):
                header[line[0]] = line[1:]
            if line and line[0] == 'DATA':
                break
        if header['DATA'][0] != 'binary':
            raise ValueError('Expected binary PCD')
        fields = []
        for i,(name,size,kind,count) in enumerate(zip(header['FIELDS'],header['SIZE'],header['TYPE'],header.get('COUNT',['1']*len(header['FIELDS'])))):
            fields.append((name if name != '_' else f'padding{i}', '<'+{'F':'f','U':'u','I':'i'}[kind]+size, (int(count),)))
        data=np.frombuffer(f.read(), dtype=np.dtype(fields))
        return np.column_stack([data[k][:,0] for k in ('x','y','z')])

def render(clouds, output, bounds=None):
    w,h=1100,760
    lo,hi = bounds if bounds is not None else (clouds[0][1].min(0),clouds[0][1].max(0))
    canvas=Image.new('RGB',(w*len(clouds),h),(12,18,26))
    for k,(label,xyz) in enumerate(clouds):
        uv=(xyz[:,[0,2]]-lo[[0,2]])/np.maximum(hi[[0,2]]-lo[[0,2]],1e-6)
        ij=np.rint(uv*np.array([w-40,h-70])+[20,20]).astype(int)
        valid=np.all((ij>=0)&(ij<[w,h]),axis=1);ij=ij[valid]
        count=np.zeros((h,w),np.int32); np.add.at(count,(h-1-ij[:,1],ij[:,0]),1)
        val=np.minimum(1.,np.log1p(count)/4.)
        rgb=np.empty((h,w,3),np.uint8)
        for c,(base,color) in enumerate(zip((12,18,26),(40,192,245))):rgb[:,:,c]=base+(color-base)*val
        canvas.paste(Image.fromarray(rgb),(w*k,0))
        ImageDraw.Draw(canvas).text((w*k+20,15),label,fill='white')
    canvas.save(output)

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('input',type=Path);ap.add_argument('output',type=Path);ap.add_argument('--crop',type=Path)
    args=ap.parse_args();xyz=read_xyz(args.input);clouds=[('Input',xyz)]
    if args.crop:clouds.append(('Crop',read_xyz(args.crop)))
    render(clouds,args.output)
    if args.crop:
        surfaces=[(label,read_xyz(Path(str(args.crop)+f'.{i}.pcd'))) for i,label in enumerate(('Low-U wall','Floor','High-U wall')) if Path(str(args.crop)+f'.{i}.pcd').exists()]
        if surfaces:render(surfaces,args.output.with_stem(args.output.stem+'-three-surfaces'),(xyz.min(0),xyz.max(0)))

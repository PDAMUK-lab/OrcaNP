import struct, sys
from PIL import Image, ImageDraw
def load(p):
    d=open(p,'rb').read(); n=struct.unpack('<I',d[80:84])[0]; tris=[]
    for i in range(n):
        v=struct.unpack('<12f',d[84+50*i:84+50*i+48]); tris.append((v[3:6],v[6:9],v[9:12]))
    return tris
tris=load(sys.argv[1]); out=sys.argv[2]; px=float(sys.argv[3]) if len(sys.argv)>3 else 6
xs=[p[0] for t in tris for p in t]; ys=[p[1] for t in tris for p in t]; zs=[p[2] for t in tris for p in t]
x0,y0,zmax=min(xs),min(ys),max(zs)
W=int((max(xs)-x0)*px)+4; H=int((max(ys)-y0)*px)+4
img=Image.new('L',(W,H),0); dr=ImageDraw.Draw(img)
# painter: draw faces sorted by mean z so higher faces cover lower ones
for t in sorted(tris,key=lambda t:sum(p[2] for p in t)):
    zc=max(p[2] for p in t)
    pts=[((p[0]-x0)*px+2,H-((p[1]-y0)*px+2)) for p in t]
    dr.polygon(pts,fill=int(40+215*zc/zmax))
img.save(out); print(out,img.size)

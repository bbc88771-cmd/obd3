import math
from PIL import Image, ImageDraw, ImageFilter

CYAN=(64,224,230); WHITE=(240,248,255); RED=(255,82,82)

def glow(layer, blur, gain=1.0):
    g = layer.filter(ImageFilter.GaussianBlur(blur))
    if gain>1: g = Image.eval(g, lambda v: min(255,int(v*gain)))
    return g

def draw_gauge(img, cx, cy, rad, lw_scale=1.0):
    N = img.size[0]
    start, sweep = 135, 270
    frac = 0.72
    gl = Image.new("RGBA", img.size, (0,0,0,0))
    gd = ImageDraw.Draw(gl)
    bb=[cx-rad,cy-rad,cx+rad,cy+rad]
    gd.arc(bb, start, start+int(sweep*frac), fill=CYAN+(255,), width=int(rad*0.16))
    img.alpha_composite(glow(gl, rad*0.08, 1.4))
    d = ImageDraw.Draw(img)
    d.arc(bb, start, start+sweep, fill=(255,255,255,45), width=int(rad*0.14))
    d.arc(bb, start, start+int(sweep*frac), fill=CYAN+(255,), width=int(rad*0.14))
    for i in range(0, sweep+1, 27):
        a=math.radians(start+i)
        r1=rad+int(rad*0.12); r2=rad+int(rad*0.26)
        d.line([(cx+r1*math.cos(a),cy+r1*math.sin(a)),(cx+r2*math.cos(a),cy+r2*math.sin(a))],
               fill=WHITE+(190,), width=max(2,int(rad*0.022)))
    a=math.radians(start+int(sweep*frac))
    nx,ny=cx+rad*0.84*math.cos(a), cy+rad*0.84*math.sin(a)
    d.line([(cx,cy),(nx,ny)], fill=RED+(255,), width=int(rad*0.07))
    hub=int(rad*0.10)
    d.ellipse([cx-hub,cy-hub,cx+hub,cy+hub], fill=RED+(255,))
    hub2=int(rad*0.045)
    d.ellipse([cx-hub2,cy-hub2,cx+hub2,cy+hub2], fill=WHITE+(255,))

def grad_bg(N, top, bot):
    img=Image.new("RGBA",(N,N),(0,0,0,0)); d=ImageDraw.Draw(img)
    for y in range(N):
        t=y/(N-1); c=tuple(int(top[i]+(bot[i]-top[i])*t) for i in range(3))
        d.line([(0,y),(N,y)], fill=c+(255,))
    return img

def render(path, size, foreground=False):
    SS=2; N=size*SS
    if foreground:
        img=Image.new("RGBA",(N,N),(0,0,0,0))
        rad=int(N*0.215)            # компактнее: вписаться в безопасную зону адаптивной иконки
    else:
        img=grad_bg(N,(14,17,22),(21,101,192))
        rad=int(N*0.30)
    draw_gauge(img, N//2, N//2, rad)
    img=img.resize((size,size), Image.LANCZOS)
    img.save(path); print("saved",path,img.size)

render("/home/user/obd3/assets/icon/app_icon.png", 1024, foreground=False)
render("/home/user/obd3/assets/icon/app_icon_foreground.png", 1024, foreground=True)

import math
from PIL import Image, ImageDraw, ImageFilter

S = 512          # итоговый размер
SS = 3           # суперсэмплинг
N = S * SS
R = int(N * 0.18)  # радиус скругления квадрата

def base(grad_top, grad_bot):
    img = Image.new("RGBA", (N, N), (0,0,0,0))
    d = ImageDraw.Draw(img)
    # вертикальный градиент
    for y in range(N):
        t = y / (N-1)
        c = tuple(int(grad_top[i] + (grad_bot[i]-grad_top[i])*t) for i in range(3))
        d.line([(0,y),(N,y)], fill=c+(255,))
    # маска скруглённого квадрата
    mask = Image.new("L", (N, N), 0)
    md = ImageDraw.Draw(mask)
    md.rounded_rectangle([0,0,N-1,N-1], radius=R, fill=255)
    out = Image.new("RGBA", (N, N), (0,0,0,0))
    out.paste(img, (0,0), mask)
    return out

def glow(layer, blur, gain=1):
    g = layer.filter(ImageFilter.GaussianBlur(blur))
    if gain>1:
        g = Image.eval(g, lambda v: min(255, int(v*gain)))
    return g

def finish(img, path):
    img = img.resize((S,S), Image.LANCZOS)
    img.save(path)
    print("saved", path)

CYAN=(64,224,230); BLUE=(33,150,243); WHITE=(240,248,255); RED=(255,82,82)

# ---------- Вариант 1: тахометр ----------
def variant_gauge():
    img = base((14,17,22),(21,101,192))
    cx, cy = N//2, N//2
    rad = int(N*0.30)
    start, sweep = 135, 270
    # слой свечения дуги
    gl = Image.new("RGBA",(N,N),(0,0,0,0))
    gd = ImageDraw.Draw(gl)
    bb=[cx-rad,cy-rad,cx+rad,cy+rad]
    gd.arc(bb, start, start+int(sweep*0.72), fill=CYAN+(255,), width=int(N*0.045))
    img.alpha_composite(glow(gl, N*0.02, 1.4))
    d = ImageDraw.Draw(img)
    # фон дуги
    d.arc(bb, start, start+sweep, fill=(255,255,255,40), width=int(N*0.04))
    # активная дуга
    d.arc(bb, start, start+int(sweep*0.72), fill=CYAN+(255,), width=int(N*0.04))
    # риски
    for i in range(0, sweep+1, 27):
        a = math.radians(start+i)
        r1=rad+int(N*0.035); r2=rad+int(N*0.075)
        d.line([(cx+r1*math.cos(a),cy+r1*math.sin(a)),
                (cx+r2*math.cos(a),cy+r2*math.sin(a))], fill=WHITE+(180,), width=max(2,int(N*0.006)))
    # стрелка
    a = math.radians(start+int(sweep*0.72))
    nx,ny = cx+int(rad*0.86)*math.cos(a), cy+int(rad*0.86)*math.sin(a)
    d.line([(cx,cy),(nx,ny)], fill=RED+(255,), width=int(N*0.018))
    d.ellipse([cx-int(N*0.03),cy-int(N*0.03),cx+int(N*0.03),cy+int(N*0.03)], fill=RED+(255,))
    d.ellipse([cx-int(N*0.013),cy-int(N*0.013),cx+int(N*0.013),cy+int(N*0.013)], fill=WHITE+(255,))
    finish(img, "/home/user/obd3/.design/icon1_gauge.png")

# ---------- Вариант 2: OBD-разъём ----------
def variant_connector():
    img = base((20,26,34),(6,9,13))
    gl = Image.new("RGBA",(N,N),(0,0,0,0))
    gd = ImageDraw.Draw(gl)
    # трапеция (D-образный 16-pin)
    topw, botw = int(N*0.52), int(N*0.40)
    top, bot = int(N*0.32), int(N*0.66)
    pts = [(N//2-topw//2, top),(N//2+topw//2, top),
           (N//2+botw//2, bot),(N//2-botw//2, bot)]
    gd.line(pts+[pts[0]], fill=CYAN+(255,), width=int(N*0.02), joint="curve")
    img.alpha_composite(glow(gl, N*0.018, 1.5))
    d = ImageDraw.Draw(img)
    d.line(pts+[pts[0]], fill=CYAN+(255,), width=int(N*0.016), joint="curve")
    # два ряда пинов
    for row,(y,w) in enumerate([(top+int(N*0.085), topw-int(N*0.10)),
                                 (bot-int(N*0.085), botw-int(N*0.06))]):
        x0 = N//2 - w//2
        for i in range(8):
            px = x0 + int(w*(i/7))
            d.ellipse([px-int(N*0.012),y-int(N*0.012),px+int(N*0.012),y+int(N*0.012)],
                      fill=BLUE+(255,))
    # пульс-линия
    yb=int(N*0.50); amp=int(N*0.05)
    seq=[(0.12,0),(0.32,0),(0.40,-amp),(0.46,amp*1.4),(0.52,-amp//2),(0.60,0),(0.88,0)]
    line=[(int(N*fx), yb+dy) for fx,dy in seq]
    d.line(line, fill=WHITE+(230,), width=int(N*0.01), joint="curve")
    finish(img, "/home/user/obd3/.design/icon2_connector.png")

# ---------- Вариант 3: авто + сигнал ----------
def variant_car():
    img = base((30,136,229),(13,71,161))
    cx, cy = N//2, int(N*0.60)
    # волны сигнала
    d = ImageDraw.Draw(img)
    for k,r in enumerate([0.20,0.28,0.36]):
        rr=int(N*r)
        bb=[cx-rr, int(N*0.30)-rr, cx+rr, int(N*0.30)+rr]
        d.arc(bb, 215, 325, fill=CYAN+(255-k*40,), width=int(N*0.016))
    # силуэт авто (вид сбоку)
    bodyy=cy
    car=[(N*0.20,bodyy),(N*0.27,bodyy-N*0.05),(N*0.38,bodyy-N*0.075),
         (N*0.44,bodyy-N*0.14),(N*0.60,bodyy-N*0.14),(N*0.66,bodyy-N*0.06),
         (N*0.80,bodyy-N*0.04),(N*0.82,bodyy),(N*0.20,bodyy)]
    d.polygon([(int(x),int(y)) for x,y in car], fill=WHITE+(255,))
    d.rectangle([int(N*0.20),int(bodyy),int(N*0.82),int(bodyy+N*0.03)], fill=WHITE+(255,))
    # колёса
    for wx in (0.34,0.68):
        r=int(N*0.055)
        d.ellipse([int(N*wx)-r,bodyy+int(N*0.01)-r,int(N*wx)+r,bodyy+int(N*0.01)+r], fill=(13,71,161,255))
        d.ellipse([int(N*wx)-r,bodyy+int(N*0.01)-r,int(N*wx)+r,bodyy+int(N*0.01)+r], outline=WHITE+(255,), width=int(N*0.012))
    finish(img, "/home/user/obd3/.design/icon3_car.png")

variant_gauge(); variant_connector(); variant_car()

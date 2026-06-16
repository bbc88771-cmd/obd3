from PIL import Image, ImageDraw, ImageFont

SS=2
W,H=720*SS,1480*SS
F="/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
FB="/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
def font(sz,b=False): return ImageFont.truetype(FB if b else F, sz*SS)

BG=(12,16,15); SURF=(23,29,28); SURF2=(33,41,39)
ACC=(255,106,44); ACC2=(45,212,191); OK=(52,211,153); WARN=(245,165,36)
ERR=(255,90,95); TXT=(242,244,243); DIM=(138,147,143)

def rr(d,box,r,fill=None,outline=None,w=1):
    d.rounded_rectangle(box, radius=r*SS, fill=fill, outline=outline, width=w*SS)

def text(d,xy,s,f,fill,anchor="la"):
    d.text((xy[0]*SS,xy[1]*SS), s, font=f, fill=fill, anchor=anchor)

def pill(d,x,y,label,color,dot=True):
    f=font(12,True); tw=d.textlength(label,font=f)
    pad=10*SS; h=24*SS; w=tw+pad*2+(14*SS if dot else 0)
    d.rounded_rectangle([x*SS,y*SS,x*SS+w,y*SS+h], radius=12*SS, fill=color+(36,))
    cx=x*SS+pad
    if dot:
        d.ellipse([cx,y*SS+h//2-3*SS,cx+6*SS,y*SS+h//2+3*SS], fill=color)
        cx+=12*SS
    d.text((cx,y*SS+h//2), label, font=f, fill=color, anchor="lm")
    return w/SS

def appbar(d,title,status,scolor):
    text(d,(24,30),title,font(21,True),TXT,"lm")
    f=font(12,True); tw=d.textlength(status,font=f)
    w=tw+34*SS
    x=W-24*SS-w
    d.rounded_rectangle([x,18*SS,x+w,42*SS], radius=12*SS, fill=scolor+(36,))
    d.ellipse([x+12*SS,26*SS,x+19*SS,33*SS], fill=scolor)
    d.text((x+24*SS,30*SS), status, font=f, fill=scolor, anchor="lm")

def navbar(d,sel):
    top=H-78*SS
    d.rectangle([0,top,W,H], fill=SURF)
    items=["Гараж","Датчики","Диагностика","Ещё"]
    n=len(items); seg=W/n
    f=font(11)
    for i,lab in enumerate(items):
        cx=seg*(i+0.5)
        col=ACC if i==sel else DIM
        if i==sel:
            d.rounded_rectangle([cx-26*SS,top+14*SS,cx+26*SS,top+40*SS],radius=14*SS,fill=ACC+(56,))
        # простая иконка-кружок
        d.ellipse([cx-7*SS,top+20*SS,cx+7*SS,top+34*SS], outline=col, width=2*SS)
        d.text((cx,top+58*SS), lab, font=f, fill=col, anchor="mm")

def metric(d,box,label,val,unit,frac,color):
    x0,y0,x1,y1=[v*SS for v in box]
    d.rounded_rectangle([x0,y0,x1,y1], radius=20*SS, fill=SURF, outline=(255,255,255,16), width=SS)
    d.ellipse([x0+14*SS,y0+14*SS,x0+26*SS,y0+26*SS], fill=color)
    d.text((x0+34*SS,y0+20*SS), label.upper(), font=font(10,True), fill=DIM, anchor="lm")
    d.text((x0+14*SS,y0+40*SS), val, font=font(25,True), fill=TXT, anchor="lt")
    vw=d.textlength(val,font=font(25,True))
    d.text((x0+14*SS+vw+6*SS, y0+58*SS), unit, font=font(12), fill=DIM, anchor="lb")
    by=y1-18*SS
    d.rounded_rectangle([x0+14*SS,by,x1-14*SS,by+6*SS], radius=3*SS, fill=SURF2)
    d.rounded_rectangle([x0+14*SS,by,x0+14*SS+(x1-x0-28*SS)*frac,by+6*SS], radius=3*SS, fill=color)

def screen_garage():
    img=Image.new("RGB",(W,H),BG); d=ImageDraw.Draw(img,"RGBA")
    appbar(d,"Revoscan","онлайн",OK)
    # connect card
    cx0,cy0,cx1,cy1=20,64,700,250
    grad=Image.new("RGB",(cx1-cx0,cy1-cy0))
    gd=ImageDraw.Draw(grad)
    for yy in range((cy1-cy0)):
        t=yy/(cy1-cy0)
        c=tuple(int((45,212,191)[i]*(1-t)*0.5+SURF[i]*(0.5+0.5*t)) for i in range(3))
        gd.line([(0,yy),(cx1-cx0,yy)],fill=c)
    grad=grad.resize(((cx1-cx0)*SS,(cy1-cy0)*SS))
    m=Image.new("L",grad.size,0); ImageDraw.Draw(m).rounded_rectangle([0,0,grad.size[0],grad.size[1]],radius=24*SS,fill=255)
    img.paste(grad,(cx0*SS,cy0*SS),m)
    d=ImageDraw.Draw(img,"RGBA")
    # bolt
    d.text((cx0*SS+18*SS, cy0*SS+18*SS),"⚡",font=font(22,True),fill=ACC2)
    d.text((cx0*SS+48*SS, cy0*SS+30*SS),"Демо-режим активен",font=font(17,True),fill=TXT,anchor="lm")
    pill(d,cx0+18,cy0+58,"ELM327",OK); pill(d,cx0+128,cy0+58,"ЭБУ",OK)
    # buttons
    by=cy0+108
    d.rounded_rectangle([cx0*SS+18*SS,by*SS,cx0*SS+430*SS,(by+44)*SS],radius=14*SS,fill=ERR)
    d.text(((cx0+18+206)*SS,(by+22)*SS),"Отключить",font=font(15,True),fill=(255,255,255),anchor="mm")
    d.rounded_rectangle([cx0*SS+446*SS,by*SS,cx1*SS-18*SS,(by+44)*SS],radius=14*SS,outline=ACC2,width=SS)
    d.text(((cx0+446+108)*SS,(by+22)*SS),"Демо",font=font(15,True),fill=ACC2,anchor="mm")
    # section
    d.rounded_rectangle([24*SS,270*SS,28*SS,286*SS],radius=2*SS,fill=ACC)
    d.text((38*SS,278*SS),"КЛЮЧЕВЫЕ ПОКАЗАТЕЛИ",font=font(12,True),fill=DIM,anchor="lm")
    # metrics 2x2
    metric(d,(20,300,356,420),"Скорость","64","км/ч",0.27,ACC2)
    metric(d,(364,300,700,420),"Обороты","2480","об/мин",0.31,ACC)
    metric(d,(20,432,356,552),"Темп. ОЖ","89","°C",0.68,WARN)
    metric(d,(364,432,700,552),"Напряжение","13.9","В",0.92,OK)
    # section diag
    d.rounded_rectangle([24*SS,572*SS,28*SS,588*SS],radius=2*SS,fill=ACC)
    d.text((38*SS,580*SS),"ДИАГНОСТИКА",font=font(12,True),fill=DIM,anchor="lm")
    # dtc card
    rr(d,[20*SS,600*SS,700*SS,710*SS],20,fill=SURF,outline=(255,255,255,16),w=1)
    d.text((38*SS,624*SS),"Ошибки (DTC)",font=font(15,True),fill=TXT,anchor="lm")
    d.text((560*SS,624*SS),"Прочитать",font=font(13),fill=ACC,anchor="lm")
    for i,(code,) in enumerate([("P0133",),("P0420",)]):
        x=38+ i*120
        rr(d,[x*SS,656*SS,(x+96)*SS,690*SS],10,fill=(120,30,30))
        d.text(((x+48)*SS,673*SS),code,font=font(13,True),fill=(255,220,220),anchor="mm")
    navbar(d,0)
    img=img.resize((720,1480),Image.LANCZOS)
    img.save("/home/user/obd3/.design/preview_garage.png"); print("garage ok")

def hubtile(d,y,icon_col,title,sub):
    rr(d,[20*SS,y*SS,700*SS,(y+72)*SS],16,fill=SURF,outline=(255,255,255,16),w=1)
    rr(d,[36*SS,(y+15)*SS,78*SS,(y+57)*SS],12,fill=icon_col+(38,))
    d.ellipse([50*SS,(y+29)*SS,64*SS,(y+43)*SS],outline=icon_col,width=2*SS)
    d.text((98*SS,(y+26)*SS),title,font=font(15,True),fill=TXT,anchor="lm")
    d.text((98*SS,(y+48)*SS),sub,font=font(12),fill=DIM,anchor="lm")
    d.text((672*SS,(y+36)*SS),"›",font=font(22),fill=DIM,anchor="mm")

def screen_diag():
    img=Image.new("RGB",(W,H),BG); d=ImageDraw.Draw(img,"RGBA")
    appbar(d,"Диагностика","онлайн",OK)
    y=80
    hubtile(d,y,ERR,"Ошибки (DTC)","Чтение и сброс кодов неисправностей"); y+=82
    hubtile(d,y,ACC2,"Стоп-кадр","Параметры в момент ошибки"); y+=82
    hubtile(d,y,OK,"Тесты на выбросы","Готовность бортовых мониторов"); y+=82
    hubtile(d,y,ACC,"Идентификаторы ЭБУ","VIN и калибровка"); y+=82
    navbar(d,2)
    img=img.resize((720,1480),Image.LANCZOS)
    img.save("/home/user/obd3/.design/preview_diag.png"); print("diag ok")

screen_garage(); screen_diag()

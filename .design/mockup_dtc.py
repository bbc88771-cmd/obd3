from PIL import Image, ImageDraw, ImageFont
SS=2; W,H=720*SS,1480*SS
F="/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"; FB="/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
def font(s,b=False): return ImageFont.truetype(FB if b else F, s*SS)
BG=(12,16,15); SURF=(23,29,28); SURF2=(33,41,39)
ACC=(255,106,44); ACC2=(45,212,191); OK=(52,211,153); WARN=(245,165,36); ERR=(255,90,95); TXT=(242,244,243); DIM=(138,147,143)
img=Image.new("RGB",(W,H),BG); d=ImageDraw.Draw(img,"RGBA")
def rr(b,r,fill=None,outline=None,w=1): d.rounded_rectangle([v*SS for v in b],radius=r*SS,fill=fill,outline=outline,width=w*SS)
def t(x,y,s,f,fill,a="la"): d.text((x*SS,y*SS),s,font=f,fill=fill,anchor=a)
# appbar
t(24,30,"Ошибки (DTC)",font(21,True),TXT,"lm")
d.ellipse([590*SS,22*SS,610*SS,42*SS],outline=DIM,width=2*SS)  # refresh icon stub
x=632; w=72
d.rounded_rectangle([x*SS,18*SS,(x+w)*SS,42*SS],radius=12*SS,fill=OK+(36,))
d.ellipse([(x+12)*SS,26*SS,(x+19)*SS,33*SS],fill=OK); t(x+24,30,"онлайн",font(12,True),OK,"lm")
# summary card
rr([20,60,700,150],20,fill=SURF,outline=ERR+(110,),w=1)
d.ellipse([38*SS,76*SS,92*SS,130*SS],fill=ERR+(38,))
t(65,103,"!",font(30,True),ERR,"mm")
t(108,86,"Check Engine горит",font(16,True),TXT,"lm")
t(108,116,"Сохранённых: 2 • ожидающих: 1 • постоянных: 1",font(12,True),DIM,"lm")
def sec(y,title,hint):
    d.rounded_rectangle([24*SS,y*SS,28*SS,(y+16)*SS],radius=2*SS,fill=ACC)
    t(38,y+8,title.upper(),font(12,True),DIM,"lm")
    t(34,y+30,hint,font(11),DIM,"lm")
def code(y,c,desc,col):
    rr([20,y,700,y+70],14,fill=SURF,outline=(255,255,255,16),w=1)
    d.rounded_rectangle([36*SS,(y+16)*SS,44*SS,(y+54)*SS],radius=4*SS,fill=col)
    t(58,y+24,c,font(16,True),TXT,"lm")
    t(58,y+48,desc,font(12),DIM,"lm")
    t(674,y+35,"›",font(22),DIM,"mm")
y=168
sec(y,"Сохранённые (2)","Подтверждённые коды — горит Check Engine"); y+=46
code(y,"P0133","Медленный отклик датчика кислорода (B1S1)",ERR); y+=78
code(y,"P0420","Эффективность катализатора ниже порога (Bank 1)",ERR); y+=92
sec(y,"Ожидающие (1)","Замечены, но ещё не подтверждены"); y+=46
code(y,"P0301","Пропуски зажигания в цилиндре 1",WARN); y+=92
sec(y,"Постоянные (1)","Сканером не стираются, гаснут сами"); y+=46
code(y,"P0420","Эффективность катализатора ниже порога (Bank 1)",ACC2); y+=92
# bottom buttons
by=H/SS-78
d.rounded_rectangle([20*SS,by*SS,460*SS,(by+48)*SS],radius=12*SS,fill=ACC)
t(240,by+24,"Пересканировать",font(15,True),(255,255,255),"mm")
d.rounded_rectangle([476*SS,by*SS,700*SS,(by+48)*SS],radius=12*SS,outline=ERR,width=SS)
t(588,by+24,"Сброс",font(15,True),ERR,"mm")
img=img.resize((720,1480),Image.LANCZOS); img.save("/home/user/obd3/.design/preview_dtc.png"); print("ok")

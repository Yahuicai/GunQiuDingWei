
import sensor, image, time,math,struct

from pyb import Pin, Timer,UART

def get_l(x,y):
    return math.sqrt((69-x)*(69-x)+(92-y)*(92-y))
tim = Timer(4, freq=20)#Frequency in Hz

#ch2 = tim.channel(2, Timer.PWM, pin=Pin("P8"), pulse_width_percent=14)
#ch1 = tim.channel(1, Timer.PWM, pin=Pin("P7"), pulse_width_percent=11)
#ch2 = tim.channel(2, Timer.PWM, pin=Pin("P8"), pulse_width_percent=14)#11-14-17
#ch1 = tim.channel(1, Timer.PWM, pin=Pin("P7"), pulse_width_percent=8)#8-11-14
Pos=38

KP_x=5
KI_x=0
KD_x=1#120#270

KP_y=KP_x
KI_y=KI_x
KD_y=KD_x#120#270



error_1_x=0.0
ei_x=0
A_x=10
B_x=5



error_1_y=0.0
ei_y=0
A_y=10
B_y=5


def get_lengthx(a,b):
    A=-0.0789
    B=-1
    C=94.945
    return int(math.fabs((A*a+B*b+C)/math.sqrt(A*A+B*B)))


def get_lengthy(a,b):
    A=25
    B=-1
    C=-3065
    return 77-int(math.fabs((A*a+B*b+C)/math.sqrt(A*A+B*B)))



def Pos_PID_control_x(Set,feedback):
    global KP_x,KD_x,KI_x,error_1_x,ei_x,A_x,B_x
    Cur_e=absCur_e=Out=0.0
    #计算偏差值
    Cur_e=Set-feedback
    #计算积分
    ei_x =ei_x+(error_1_x+Cur_e)*0.09
    #取绝对值
    absCur_e=math.fabs(Cur_e)
    #根据偏差值限制积分所带来的影响
    if(absCur_e<=B_x):
        f=1
    elif(absCur_e>B_x and absCur_e<=A_x+B_x):
        f = (A_x - absCur_e +B_x)/A_x
    else:
        f = 0
    #I积分最大值
    ei_max_x=8000
    if(ei_x > ei_max_x):
        ei_x = ei_max_x
    elif(ei_x<-ei_max_x):
        ei_x=-ei_max_x
    #主要公式
    Out = KP_x * Cur_e + (KD_x) * (Cur_e - error_1_x) / 0.04 + KI_x * f * ei_x#0.04
    #输出限制
    Out_MAX=1200
    if(Out>Out_MAX):
        Out=Out_MAX
    if(Out<-Out_MAX):
        Out=-Out_MAX
    return -Out/100*0.25

def Pos_PID_control_y(Set,feedback):
    global KP_y,KD_y,KI_y,error_1_y,ei_y,A_y,B_y
    Cur_e=absCur_e=Out=0.0
    #计算偏差值
    Cur_e=Set-feedback
    #计算积分
    ei_y =ei_y+(error_1_y+Cur_e)*0.09
    #取绝对值
    absCur_e=math.fabs(Cur_e)
    #根据偏差值限制积分所带来的影响
    if(absCur_e<=B_y):
        f=1
    elif(absCur_e>B_y and absCur_e<=A_y+B_y):
        f = (A_y - absCur_e +B_y)/A_y
    else:
        f = 0
    #I积分最大值
    ei_max_y=8000
    if(ei_y > ei_max_y):
        ei_y = ei_max_y
    elif(ei_y<-ei_max_y):
        ei_y=-ei_max_y
    #主要公式
    Out = KP_y * Cur_e + (KD_y) * (Cur_e - error_1_y) / 0.04 + KI_y * f * ei_y#0.04
    #输出限制
    Out_MAX=1200
    if(Out>Out_MAX):
        Out=Out_MAX
    if(Out<-Out_MAX):
        Out=-Out_MAX
    return -Out/100*0.25




def floatToBytes(f):
    bs = struct.pack("f",f)
    return (bs[3],bs[2],bs[1],bs[0])
def niming_report(fun,data,a,b,c,le):
    send_buf=[1]*32
    i=0
    if(le>28):
        return
    send_buf[le+3]=0
    send_buf[0]=0X88
    send_buf[1]=fun
    send_buf[2]=le

    temp=floatToBytes(a)
    send_buf[3]=temp[0]
    send_buf[4]=temp[1]
    send_buf[5]=temp[2]
    send_buf[6]=temp[3]

    temp=floatToBytes(b)
    send_buf[7]=temp[0]
    send_buf[8]=temp[1]
    send_buf[9]=temp[2]
    send_buf[10]=temp[3]

    temp=floatToBytes(c)
    send_buf[11]=temp[0]
    send_buf[12]=temp[1]
    send_buf[13]=temp[2]
    send_buf[14]=temp[3]

    for i in range(0,le+3):
        send_buf[le+3]=send_buf[le+3]+send_buf[i]
    for i in range(0,le+4):
        uart.writechar(send_buf[i])
# 为了使色彩追踪效果真的很好，你应该在一个非常受控制的照明环境中。
Q=4
R=100
Kg_L=0.0
X_L=0.0
XT_L=[1]*2
XL_covariance=10.0
XTL_covariance=10.0
def KalmanFilter(value):
    global Kg_L,X_L,X_TL,XL_covariance,XTL_covariance,Q,R
    X_L= XT_L[0]
    XL_covariance = math.sqrt(XTL_covariance * XTL_covariance + Q * Q)
    Kg_L = math.sqrt(XL_covariance * XL_covariance /(XL_covariance * XL_covariance + R * R))
    XT_L[1] = X_L + Kg_L * (value-X_L)
    XTL_covariance = math.sqrt((1-Kg_L) * XL_covariance * XL_covariance)
    XT_L[0] = XT_L[1]
    return XT_L[1]
green_threshold   = (80, 22, 127, 35, -128, 127)
#设置绿色的阈值，括号里面的数值分别是L A B 的最大值和最小值（minL, maxL, minA,
# maxA, minB, maxB），LAB的值在图像左侧三个坐标图中选取。如果是灰度图，则只需
#设置（min, max）两个数字即可。

# You may need to tweak the above settings for tracking green things...
# Select an area in the Framebuffer to copy the color settings.
l=0
sensor.reset() # Initialize the camera sensor.
sensor.set_pixformat(sensor.RGB565) # use RGB565.
sensor.set_framesize(sensor.QQVGA) # use QQVGA for speed.
sensor.skip_frames(10) # Let new settings take affect.
sensor.set_auto_whitebal(False) # turn this off.
#关闭白平衡。白平衡是默认开启的，在颜色识别中，需要关闭白平衡。
clock = time.clock() # Tracks FPS.


uart = UART(3, 115200, timeout_char=1000)
uart.init(115200, bits=8, parity=None, stop=1, timeout_char=1000)
p1 = Pin('P7') # P7 has TIM4, CH1
#p2 = Pin('P8') # P7 has TIM4, CH1
tim = Timer(4, freq=200 )
ch1 = tim.channel(1, Timer.PWM, pin=p1)
#ch2 = tim.channel(2, Timer.PWM, pin=p2)
ch1.pulse_width_percent(15.0)
#ch2.pulse_width_percent(15.0)
while(True):
    clock.tick() # Track elapsed milliseconds between snapshots().
    img = sensor.snapshot() # Take a picture and return the image.
    #uart.write('abc')

    blobs = img.find_blobs([green_threshold])
    cout = 0
    Out_old=0

    if blobs:
    #如果找到了目标颜色
        for b in blobs:
        #迭代找到的目标颜色区域
            # Draw a rect around the blob.
            #用矩形标记出目标颜色区域
            img.draw_rectangle(b[0:4]) # rect

            #在目标颜色区域的中心画十字形标记
            img.draw_cross(b[5], b[6]) # cx, cy
            #根据图像坐标算出白板上xy坐标
            x=get_lengthx(b[5],b[6])
            y=get_lengthy(b[5],b[6])
            P_x=Pos_PID_control_x(28,x)
            P_y=Pos_PID_control_y(28,y)
            ch2 = tim.channel(2, Timer.PWM, pin=Pin("P8"), pulse_width_percent=14+P_y)
            ch1 = tim.channel(1, Timer.PWM, pin=Pin("P7"), pulse_width_percent=11-P_x)
            print("x=",x,",y=",y)
            #l=get_l(b[5],b[6])
            #kx=x_KalmanFilter(x)
            #ky=y_KalmanFilter(x)
            #打印PID的值
            print(P_x,P_y)
            #匿名上位机接收软件
            niming_report(0xA1,0,x,P_x,y,28)



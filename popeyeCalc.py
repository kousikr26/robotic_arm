import math
import serial
def radtheta(x,y):
    rad=math.sqrt((x*x)+(y*y))
    balloon_height=5.0
    #sides of triangle
    c=math.sqrt((rad*rad)+(balloon_height*balloon_height))
    a=25
    b=25
    s=float((a+b+c))/2
    area=math.sqrt(s*(s-a)*(s-b)*(s-c))
    R=(a*b*c)/(4*area)
    #angles of triangle
    alpha=math.atan(balloon_height/c)
    A=math.asin(a/(2*R))
    B=math.asin(b/(2*R))
    C=math.asin(c/(2*R))
    #final angles for arm
    #base_arm=a, second_arm=b/
    if y==0:
        y=0.000001
    if y>0:
        base_arm_angle=int(math.degrees(B+alpha))
    elif y<0:
        base_arm_angle=180-int(math.degrees(B+alpha))
    second_arm_angle=int(math.degrees(C))
    if x!=0:
        theta=int(math.degrees(math.atan(float(y)/x)))
    else :
        theta=90        
    if theta<0:
        theta=theta+180
    if theta==0 and (x*y)<0:
        theta = 180
    elif theta==0 and (x*y)>0:
        theta = 0
    return theta,base_arm_angle,second_arm_angle

def arduino(angle1,angle2,angle3):
    ser= serial.Serial()
    ser.baudrate=9600
    ser.port="COM3"
    ser.open();
    final_string="<"+str(angle1)+" "+str(angle2)+" "+str(angle3)+">"
    ser.write(final_string.encode())
    ser.close()

def error_detection(theta, dist, angle1, angle2):
    x,y = calc_arm_details()
    r = math.sqrt((x*x)+(y*y))
    angle_arm = int(math.degrees(math.atan(y/x)))
    count=0
    while angle_arm-theta>2 or angle_arm-theta<2 :
        theta+=int((theta-angle_arm)*0.75)
        arduino(theta,angle1,angle2)
        count+=1
        if count==5:
            break
    count=0
    while r-dist>4 or r-dist<4 :
        if r-dist>1 :
            angle1+=2
            angle2-=2
            arduino(theta,angle1,angle2)
        if r-dist<1 :
            angle2+=2
            angle1-=2
            arduino(theta,angle1,angle2)
        count+=1
        if count==5:
            break
    '''ser=serial.Serial()
    ser.baudrate=9600
    ser.port="COM3"
    ser.open();
    ser.write("1".encode())
    ser.close()'''
    

    
    

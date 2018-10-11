#red,blue,green,yellow
"""
Problems/to-do:
1.When  same colour balls are touching single object is taken
2.COM port
3.height-y coordinate IMPORTANT:HERE ORIGIN IS KEPT AS TOP LEFT CORNER IN ROUND 1 IT IS CHANGED TO BOTTOM RIGHT
4.final string
5.angle_arm = int(math.degrees(math.atan(y/x))) x,y in 3rd quad &1st quad will be taken same
6.Change a,b
"""
"""TO change:
HSV VALUES for blue
retval,threshold=cv2.threshold(grayimg,30,255,cv2.THRESH_BINARY_INV)#CHECK THRESHOLD VALUE
if radius>20:#CHANGE THIS VALUE ACCORDING TO CAMERA HEIGHT AND RADIUS
if cv2.contourArea(i)>50:
if len(approx)>=7:#CHeck for diffent values higher than 5
balloon_height=1.0
time.sleep(15)
"""
print("start")
import cv2
import numpy as np
import math
import serial
import time
import urllib.request

url="http://192.168.43.1:8080/shot.jpg"
#img=cv2.imread('ball2.jpg',-1)
targets=[]
#gray=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
#img_hsv=cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
#grayscale=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

ser= serial.Serial()
ser.baudrate=9600
ser.port="COM5"
ser.open();


kernel=np.ones((5,5),np.uint8)
def dist(x1,y1,x2,y2):
    return ((x2-x1)**2+(y2-y1)**2)**(0.5)
def findfactor(grayimg):
    
    retval,threshold=cv2.threshold(grayimg,30,255,cv2.THRESH_BINARY_INV)#CHECK THRESHOLD VALUE
    #median=cv2.medianBlur(threshold,15)
    threshold=cv2.erode(threshold,kernel,iterations=3)
    threshold=cv2.dilate(threshold,kernel,iterations=1)
    cv2.imshow('thresh',threshold)
    img_factor,contours_factor,hierarchy_factor=cv2.findContours(threshold,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(img,contours_factor,-1,(255,255,255),3)#USE A BETTER ALGORITHM and remove this line img may not be defined
    cv2.imshow('contours',img)
##    try:
##        cv2.drawContours(img,contours_factor,-1,(255,0,0),3)#USE A BETTER ALGORITHM and remove this line img may not be defined
##        cv2.imshow('img',img)
##    except:
##        pass
    squares=[]
    
    for i in contours_factor:
        #approx = cv2.approxPolyDP(i,0.01*cv2.arcLength(i,True),True)
        #check if contour is square if dont use number of edges or use it after eroding and/or dilating to avoid partial square
        if cv2.contourArea(i)>0:
            M_factor=cv2.moments(i)
            cx_factor = int(M_factor['m10']/M_factor['m00'])
            cy_factor = int(M_factor['m01']/M_factor['m00'])
            cv2.circle(img,(cx_factor,cy_factor), 5, (255,255,255), -1)   
            squares.append([cx_factor,cy_factor])
        else:
            return 0,0,0
    if len(squares)==4:#EXCLUDING STARTING SQUARE
        distances=[dist(squares[0][0],squares[0][1],squares[1][0],squares[1][1]),dist(squares[0][0],squares[0][1],squares[2][0],squares[2][1]),dist(squares[0][0],squares[0][1],squares[3][0],squares[3][1])]
        diagonal=max(distances)
        sqnum=distances.index(diagonal)+1
        centerx=(squares[sqnum][0]+squares[0][0])/2
        centery=(squares[sqnum][1]+squares[0][1])/2
        cv2.circle(img,(int(centerx),int(centery)),5,(255,255,255),-1)
        factor=127.26/diagonal#length of diagonal in cm
        return factor,centerx,centery
    else:
        return 0,0,0
        print("Extra or less black squares found")#CHECK

def findcenter(colmask):
    funcvalue=[]
    img2,contours,hierarchy=cv2.findContours(colmask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    #cv2.drawContours(img,contours,-1,(203,192,203),3)
    if len(contours)>0:
        
        for i in contours:
            #print(cv2.contourArea(i))
            ((x, y), radius) = cv2.minEnclosingCircle(i)
            if radius>30:#CHANGE THIS VALUE ACCORDING TO CAMERA HEIGHT AND RADIUS
                
                approx = cv2.approxPolyDP(i,0.01*cv2.arcLength(i,True),True)
                
                if len(approx)>=7:#CHeck for diffent values higher than 5
                    
                    
                    M=cv2.moments(i)
                    
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    x=(cx-centerx)*factor
                    y=(-1)*(cy-centery)*factor#ORIGIN Is TOP LEFT

                    cv2.circle(img,(cx,cy), 5, (255,255,255), -1)
                    funcvalue.append([x,y])
                    #Also check for area of moment afterwards
                else:
                    pass
            else:
                pass
    else:
        print("NO CONTOURS")
    return funcvalue    


def radtheta(x,y):
    rad=math.sqrt((x*x)+(y*y))
    balloon_height=0
    #sides of triangle
    c=math.sqrt((rad*rad)+(balloon_height*balloon_height))
    a=35
    b=32
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
        second_arm_angle=90-int(math.degrees(C))
    elif y<0:
        base_arm_angle=180-int(math.degrees(B+alpha))
        second_arm_angle=90+int(math.degrees(C))
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
def arduino(angle1,angle2,angle3,delay):
    
    final_string="<"+str(angle1)+","+str(angle3)+","+str(angle2)+">"
    time.sleep(2)
    ser.write(final_string.encode())
    time.sleep(delay)


    
def calc_arm_details():
    img=takeimage()
    img_hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    lower_purple=np.array([120,140,0])
    upper_purple([140,255,255])
    purple_mask=cv2.inRange(img_hsv,lower_purple,upper_purple)
    purple_mask=cv2.erode(purple_mask,kernel,iterations=2)
    purple_mask=cv2.dilate(purple_mask,kernel,iterations=2)
    img2,contours,hierarchy=cv2.findContours(colmask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    if len(contours)>0:
        marker=max(contours,key=cv2.contourArea)


        
        M=cv2.moments(marker)
        
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        x=(cx-centerx)*factor
        y=(-1)*(cy-centery)*factor#ORIGIN Is now center

        return x,y

        
    else:
        print("marker not detected")
def pop(theta,angle1,angle2):
    if angle1<=90:
        arduino(theta,angle1+10,angle2,0.5)
        arduino(theta,angle1-15,angle2,0.5)
        arduino(theta,angle1+15,angle2,0.5)
    else:
        arduino(theta,angle1-10,angle2,0.5)
        arduino(theta,angle1+15,angle2,0.5)
        arduino(theta,angle1+15,angle2,0.5)
def error_detection(theta,angle1, angle2):
    print("angle0",theta,"angle1",angle1,"angle2",angle2)
    arduino(theta,angle1,angle2,4)
    pop(theta,angle1,angle2)
##    x,y = calc_arm_details()
##    r = math.sqrt((x*x)+(y*y))
##    angle_arm = int(math.degrees(math.atan(y/x)))
##    count=0
##    while angle_arm-theta>2 or angle_arm-theta<2 :
##        theta+=int((theta-angle_arm)*0.75)
##        arduino(theta,angle1,angle2)
##        count+=1
##        if count==5:
##            break
##    count=0
##    while r-dist>4 or r-dist<4 :
##        if r-dist>1 :
##            angle1+=2
##            angle2-=2
##            arduino(theta,angle1,angle2)
##        if r-dist<1 :
##            angle2+=2
##            angle1-=2
##            arduino(theta,angle1,angle2)
##        count+=1
##        if count==5:
##            break
##    '''ser=serial.Serial()
##    ser.baudrate=9600
##    ser.port="COM3"
##    ser.open();
##    ser.write("1".encode())
##    ser.close()'''
def takeimage():
    imgResp = urllib.request.urlopen(url)
    imgNp = np.array(bytearray(imgResp.read()),dtype=np.uint8)
    img = cv2.imdecode(imgNp,-1)
    return img
def findballoons():
    img=takeimage()
    img_hsv=cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #red
    lower_red = np.array([0,50,50])#HSV VAUES WILL BE DIFFERENT
    upper_red = np.array([10,255,255])
    mask0 = cv2.inRange(img_hsv, lower_red, upper_red)
    #cv2.imshow('mask0',mask0)
    lower_red = np.array([170,50,50])
    upper_red = np.array([180,255,255])
    mask1 = cv2.inRange(img_hsv, lower_red, upper_red)
    #cv2.imshow('mask1',mask1)

    redmask = mask0+mask1

    redmask=cv2.erode(redmask,kernel,iterations=2)#use different iterations and kernels
    redmask=cv2.dilate(redmask,kernel,iterations=2)
    #redmask=cv2.erode(redmask,kernel,iterations=4)
    cv2.imshow('r',redmask)
    red_center=findcenter(redmask)
    targets.append(red_center)



    #blue
    lower_blue=np.array([80,100,0])#Check blue value doesnt conflict with background
    upper_blue=np.array([140,255,255])
    bluemask=cv2.inRange(img_hsv,lower_blue,upper_blue)#HSV VAUES WILL BE DIFFERENT

    bluemask=cv2.erode(bluemask,kernel,iterations=2)
    bluemask=cv2.dilate(bluemask,kernel,iterations=2)
    #cv2.filter2D(bluemask,-1,kernel)
    blue_center=findcenter(bluemask)
    targets.append(blue_center)
    cv2.imshow('b',bluemask)

    #green
    lower_green=np.array([41,0,0])#HSV VAUES WILL BE DIFFERENT
    upper_green=np.array([80,255,255])
    greenmask=cv2.inRange(img_hsv,lower_green,upper_green)
    greenmask=cv2.erode(greenmask,kernel,iterations=2)
    greenmask=cv2.dilate(greenmask,kernel,iterations=2)
    green_center=findcenter(greenmask)
    targets.append(green_center)
    cv2.imshow('g',greenmask)

    #yellow
    lower_yellow=np.array([18,85,64])#HSV VAUES WILL BE DIFFERENT
    upper_yellow=np.array([40,255,255])
    yellowmask=cv2.inRange(img_hsv,lower_yellow,upper_yellow)
    yellowmask=cv2.erode(yellowmask,kernel,iterations=2)
    yellowmask=cv2.dilate(yellowmask,kernel,iterations=2)
    yellow_center=findcenter(yellowmask)
    targets.append(yellow_center)
    cv2.imshow('y',yellowmask)
    return targets
factor=0  


while factor==0:
    img=takeimage()
    grayscale=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    print("detecting black squares")

    factor,centerx,centery=findfactor(grayscale)#grayscale image


print("black detected")
targets=findballoons()
print(targets)
for color in targets:
    for balloon in color:
        #targetdist=dist(centerx*factor,centery*factor,balloon[0],balloon[1])
        print("balloon dist",targets.index(color))#CHECK THIS VALUE
        if (balloon[0]>=0 and balloon[1]>=0) or (balloon[0]<0 and balloon[1]>=0):
            theta,base_arm_angle,second_arm_angle=radtheta(balloon[0],balloon[1])
            error_detection(theta,base_arm_angle,second_arm_angle)
            error_detection(theta,180-base_arm_angle,180-second_arm_angle)
        else:
            print("already popped")
            pass
        
"""code for opposite balloon
if |checkradius-curradius|<5 and math.degrees(|degrees(angle(newballloon))-(180+theta)|)<5 or math.degrees(|degrees(angle(newballloon))-(theta-180)|)<5

add new and old balloon to check list
check if each balloon is not in checklist befor popping it
"""
arduino(0,90,90,1)
ser.close()

print("complete")
cv2.imshow('img',img)
cv2.waitKey(0)
cv2.destroyAllWindows()


import math
import serial
import cv2
import numpy as np

img=cv2.imread('round1new.jpg',-1)
#gray=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)#HERE GRAY IS NOT GRAYSCALE IT IS HSV
grayscale=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
kernel=np.ones((5,5),np.uint8)
height,width,channels=img.shape
print(height,width)
def dist_calc(x1,y1,x2,y2):
    return ((x2-x1)**2+(y2-y1)**2)**(0.5)
def findfactor(grayimg):
    
    retval,threshold=cv2.threshold(grayimg,5,255,cv2.THRESH_BINARY_INV)#CHECK THRESHOLD VALUE
    #median=cv2.medianBlur(threshold,15)
    threshold=cv2.erode(threshold,kernel,iterations=3)
    #dilation=cv2.dilate(threshold,kernel,iterations=1)
    #cv2.imshow('thresh',threshold)
    img_factor,contours_factor,hierarchy_factor=cv2.findContours(threshold,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
##    try:
##        cv2.drawContours(img,contours_factor,-1,(255,0,0),3)#USE A BETTER ALGORITHM and remove this line img may not be defined
##        #cv2.imshow('img',img)
##    except:
##        print("img may not be defined")
##        pass

    squares=[]
    
    for i in contours_factor:
        approx = cv2.approxPolyDP(i,0.01*cv2.arcLength(i,True),True)
        if len(approx)==4:
            M_factor=cv2.moments(i)
            cx_factor = int(M_factor['m10']/M_factor['m00'])
            cy_factor = int(M_factor['m01']/M_factor['m00'])
            cv2.circle(img,(cx_factor,cy_factor), 5, (255,0,0), -1)   
            squares.append([cx_factor,cy_factor])
        else:
            pass
    if len(squares)==4:#EXCLUDING STARTING SQUARE
        distances=[dist_calc(squares[0][0],squares[0][1],squares[1][0],squares[1][1]),dist_calc(squares[0][0],squares[0][1],squares[2][0],squares[2][1]),dist_calc(squares[0][0],squares[0][1],squares[3][0],squares[3][1])]
        diagonal=max(distances)
        sqnum=distances.index(diagonal)+1
        centerx=(squares[sqnum][0]+squares[0][0])/2
        centery=(squares[sqnum][1]+squares[0][1])/2
        factor=127.26/diagonal#length of diagonal in cm
        return factor,centerx,centery
    else:
        print("Extra or less black squares found")#CHECK

        
factor,centerx,centery=findfactor(grayscale)#grayscale image


#-------------------------------------------------------------------------
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
##def calc_arm_details():
##    return 1,2#COMPLETE
def error_detection(theta, dist, angle1, angle2):
    x,y = calc_arm_details()
    r = math.sqrt((x*x)+(y*y))
    angle_arm = int(math.degrees(math.atan(y/x)))
    count=0
    while angle_arm-theta>2 or angle_arm-theta<-2 :
        theta+=int(theta-angle_arm)
        arduino(theta,angle1,angle2)
        count+=1
        if count==5:
            break
    count=0
    while r-dist>2 or r-dist<-2 :
        if r-dist>2 :
            angle1+=2
            angle2-=2
            arduino(theta,angle1,angle2)
        if r-dist<-2 :
            angle2+=2
            angle1-=2
            arduino(theta,angle1,angle2)
        count+=1
        if count==5:
            break
    '''ser=serial.Serial()
    ser.baudrate=9600
    ser.port="COM3"
    ser.open()
    ser.write("1".encode())
    ser.close()'''
#-------------------------------------------------------------------------
def findsquares():#requires img
    
    img_hsv=cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lower_red = np.array([0,50,50])
    upper_red = np.array([10,255,255])
    mask0 = cv2.inRange(img_hsv, lower_red, upper_red)

    lower_red = np.array([170,50,50])
    upper_red = np.array([180,255,255])
    mask1 = cv2.inRange(img_hsv, lower_red, upper_red)
    mask = mask0+mask1
    #output = img.copy()
    #output[np.where(mask==0)] = 0


    img2,contours,hierarchy=cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(img,contours,-1,(255,0,0),3)
    targets=[]
    for i in contours:
        approx = cv2.approxPolyDP(i,0.01*cv2.arcLength(i,True),True)
        if len(approx)==4:
            #cnt=contours[0]
            M=cv2.moments(i)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            x=(cx-centerx)*factor
            y=(-1)*(cy-centery)*factor
            cv2.circle(img,(cx,cy), 5, (0,0,0), -1)
            targets.append([x,y])#x,y in cm
            
            print("coordinates in pixels",cx,height-cy)
            print("xy",x,y)
            print("angle",math.degrees(math.atan(y/x)))
            print("center",centerx,height-centery)
            print("radius in cm",dist_calc(cx,cy,centerx,height-centery)*factor)
        else:
            pass
    cv2.imshow('img',img)
    return targets
targets=findsquares()
if len(targets)==2:
    for i in targets:
        theta,base_arm_angle,second_arm_angle=radtheta(i[0],i[1])
        dist=dist_calc(i[0],i[1],0,0)
        error_detection(theta,dist,base_arm_angle,second_arm_angle)
else:
    print("2 red squares not detected")
cv2.imshow('img',img)
#cv2.imshow('mask',mask)

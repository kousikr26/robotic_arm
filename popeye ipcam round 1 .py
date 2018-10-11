import math
import serial
import cv2
import numpy as np
import time
import urllib.request
"""possible probs:
contour area may be differnt for given camera
Starting black square detection
check 'm00' nnot zero
break added may cause images to not show up
"""


"""changes:
3 dilates after 3 erodes in findfactor
y=(-1)*(cy-centery)*factor
        print("coordinates in pixels",cx,height-cy)
        print("radius in cm",dist_calc(cx,cy,centerx,height-centery)*factor)

added dist argument in error detection and its call"""
"""Xaxis is horizontal y axis is inverted vertical from top left corner"""


url="http://192.168.43.1:8080/shot.jpg"




#img=cv2.imread('1.jpg',-1)
ser= serial.Serial()
ser.baudrate=9600
ser.port="COM5"                                 
ser.open()

cap=cv2.VideoCapture(1)

#gray=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)#HERE GRAY IS NOT GRAYSCALE IT IS HSV

#cv2.imshow('gray',grayscale)
kernel=np.ones((5,5),np.uint8)
def dist_calc(x1,y1,x2,y2):
    return ((x2-x1)**2+(y2-y1)**2)**(0.5)
def findfactor(grayimg):
    
    retval,threshold=cv2.threshold(grayimg,50,255,cv2.THRESH_BINARY_INV)#CHECK THRESHOLD VALUE
    #median=cv2.medianBlur(threshold,15)
    threshold=cv2.erode(threshold,kernel,iterations=3)
    threshold=cv2.dilate(threshold,kernel,iterations=1)
    
    cv2.imshow('thresh',threshold)
    img_factor,contours_factor,hierarchy_factor=cv2.findContours(threshold,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(img,contours_factor,-1,(255,255,255),3)
    cv2.imshow('contours',img)
##    try:
##        cv2.drawContours(img,contours_factor,-1,(255,0,0),3)#USE A BETTER ALGORITHM and remove this line img may not be defined
##        #cv2.imshow('img',img)
##    except:
##        print("img may not be defined")
##        pass

    squares=[]
    
    for i in contours_factor:
        #approx = cv2.approxPolyDP(i,0.01*cv2.arcLength(i,True),True)
        if cv2.contourArea(i)>0:#CHANGE AREA
            #print("In contour area check")
            M_factor=cv2.moments(i)
            
        
            
            cx_factor = int(M_factor['m10']/M_factor['m00'])
            cy_factor = int(M_factor['m01']/M_factor['m00'])
            
            
            cv2.circle(img,(cx_factor,cy_factor), 5, (255,255,255), -1)   
            squares.append([cx_factor,cy_factor])
           # print("blacksquares",squares)
            cv2.imshow('img',img)
        else:
            return 0,0,0
    if len(squares)==4:#EXCLUDING STARTING SQUARE
        distances=[dist_calc(squares[0][0],squares[0][1],squares[1][0],squares[1][1]),dist_calc(squares[0][0],squares[0][1],squares[2][0],squares[2][1]),dist_calc(squares[0][0],squares[0][1],squares[3][0],squares[3][1])]
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
        
        



#-------------------------------------------------------------------------
def radtheta(x,y):
    rad=math.sqrt((x*x)+(y*y))
    balloon_height=1.0
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

def arduino(angle1,angle2,angle3):

    final_string="<"+str(angle1)+","+str(angle3)+","+str(angle2)+">"
    time.sleep(2)
    ser.write(final_string.encode())
    
    time.sleep(2)

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
def error_detection(theta,angle1, angle2):
    print("angle0",theta,"angle1",angle1,"angle2",angle2)#remove these two lines when marker is added
    arduino(theta,angle1,angle2)
##    x,y = calc_arm_details()
##    r = math.sqrt((x*x)+(y*y))
##    angle_arm = int(math.degrees(math.atan(y/x)))
##    count=0
##    while angle_arm-theta>2 or angle_arm-theta<-2 :
##        theta+=int(theta-angle_arm)
##        arduino(theta,angle1,angle2)
##        count+=1
##        if count==5:
##            break
##    count=0
##    while r-dist>2 or r-dist<-2 :
##        if r-dist>2 :
##            angle1+=2
##            angle2-=2
##            arduino(theta,angle1,angle2)
##        if r-dist<-2 :
##            angle2+=2
##            angle1-=2
##            arduino(theta,angle1,angle2)
##        count+=1
##        if count==5:
##            break
    '''ser=serial.Serial()
    ser.baudrate=9600
    ser.port="COM3"
    ser.open()
    ser.write("1".encode())
    ser.close()'''
#-------------------------------------------------------------------------
def findsquares():#requires img
    img=takeimage()
    height,width,channels=img.shape
    img_hsv=cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lower_red = np.array([0,32,50])
    upper_red = np.array([10,255,255])
    mask0 = cv2.inRange(img_hsv, lower_red, upper_red)

    lower_red = np.array([170,32,50])
    upper_red = np.array([180,255,255])
    mask1 = cv2.inRange(img_hsv, lower_red, upper_red)
    mask = mask0+mask1
    #output = img.copy()
    #output[np.where(mask==0)] = 0
##    lower_blue=np.array([80,100,0])#Check blue value doesnt conflict with background
##    upper_blue=np.array([140,255,255])
    #mask=cv2.inRange(img_hsv,lower_red,upper_red)#HSV VAUES WILL BE DIFFERENT
    mask=cv2.erode(mask,kernel,iterations=2)
    mask=cv2.dilate(mask,kernel,iterations=2)
    cv2.imshow('bluemask',mask)
    


    img2,contours,hierarchy=cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(img,contours,-1,(255,255,255),3)
    targets=[]
    cv2.imshow('sq',img)
    a=max(contours,key=cv2.contourArea)
    #del contours[contours.index(a)]
    #b=max(contours,key=cv2.contourArea)
    #del contours[contours.index(a)]
    contours=[a]
    for i in contours:
        #approx = cv2.approxPolyDP(i,0.01*cv2.arcLength(i,True),True)
        #if len(approx)==4:
            #cnt=contours[0]
    
        M=cv2.moments(i)
        
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])

            
        x=(cx-centerx)*factor
        y=(-1)*(cy-centery)*factor
        print("xy",x,y)
        cv2.circle(img,(cx,cy), 5, (255,255,255), -1)
        targets.append([x,y])#x,y in cm
            
        #print("coordinates in pixels",cx,height-cy)
        #print("coordinates in cm",cx*factor,(height-cy)*factor)
        print("radius in cm",dist_calc(cx,cy,centerx,height-centery)*factor)
    #else:
    #    pass
   
        
    return targets
def takeimage():
    #imgResp = urllib.request.urlopen(url)
    #imgNp = np.array(bytearray(imgResp.read()),dtype=np.uint8)
    #img = cv2.imdecode(imgNp,-1)
    #return img
    ret,frame=cap.read()
    return frame
factor=0  
while True:
    
    while factor==0:
        img=takeimage()
        print("detecting black")
        grayscale=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)


        factor,centerx,centery=findfactor(grayscale)#grayscale image

    print("factor",factor)
    
    
    targets=findsquares()
    
    if len(targets)==1:
        
        cv2.imshow('i',img)
        for i in targets:
            theta,base_arm_angle,second_arm_angle=radtheta(i[0],i[1])
            #targetdist=calc_dist(centerx*factor,centery*factor,i[0],i[1])
            error_detection(theta,base_arm_angle,second_arm_angle)
            if base_arm_angle<=90:
                error_detection(theta,base_arm_angle+10,second_arm_angle)
            else:
                error_detection(theta,base_arm_angle-10,second_arm_angle)
            error_detection(theta,180-base_arm_angle,180-second_arm_angle)
            if base_arm_angle<=90:
                error_detection(theta,180-base_arm_angle-10,180-second_arm_angle)
            else:
                error_detection(theta,180-base_arm_angle+10,180-second_arm_angle)
        break
    else:
        print("detected",len(targets))
        print("2 red squares not detected")
    

    #cv2.imshow('mask',mask)
time.sleep(5)
arduino(0,90,90)
cap.release()
ser.close()
print("complete")

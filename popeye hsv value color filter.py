import cv2
import numpy as np
import urllib.request
import time
time.sleep(7)
kernel=np.ones((5,5),np.uint8)
url="http://192.168.43.1:8080/shot.jpg"
#imgResp = urllib.request.urlopen(url)
#imgNp = np.array(bytearray(imgResp.read()),dtype=np.uint8)
#frame = cv2.imdecode(imgNp,-1)
frame=cv2.imread("real.jpg",-1)
#cap=cv2.VideoCapture(1)
#ret,frame=cap.read()
img_hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)#hue saturation value

targets=[]
centerx=0
centery=0
factor=1
def findfactor(grayimg):
    
    retval,threshold=cv2.threshold(grayimg,70,255,cv2.THRESH_BINARY_INV)#CHECK THRESHOLD VALUE
    #median=cv2.medianBlur(threshold,15)
    threshold=cv2.erode(threshold,kernel,iterations=5)
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
    if len(squares)==5:#EXCLUDING STARTING SQUARE
        distances=[dist_calc(squares[0][0],squares[0][1],squares[1][0],squares[1][1]),dist_calc(squares[0][0],squares[0][1],squares[2][0],squares[2][1]),dist_calc(squares[0][0],squares[0][1],squares[3][0],squares[3][1]),dist_calc(squares[0][0],squares[0][1],squares[4][0],squares[4][1])]
        distances2=[dist_calc(squares[1][0],squares[1][1],squares[0][0],squares[0][1]),dist_calc(squares[1][0],squares[1][1],squares[2][0],squares[2][1]),dist_calc(squares[1][0],squares[1][1],squares[3][0],squares[3][1]),dist_calc(squares[1][0],squares[1][1],squares[4][0],squares[4][1])]
        alldistances=distances+distances2
        diagonal=max(alldistances)
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

                    cv2.circle(frame,(cx,cy), 5, (255,255,255), -1)
                    funcvalue.append([x,y])
                    #Also check for area of moment afterwards
                else:
                    pass
            else:
                pass
    else:
        print("NO CONTOURS")
    return funcvalue  



lower_red = np.array([0,50,0])
upper_red = np.array([10,255,255])
mask0 = cv2.inRange(img_hsv, lower_red, upper_red)

lower_red = np.array([170,50,0])
upper_red = np.array([180,255,255])
mask1 = cv2.inRange(img_hsv, lower_red, upper_red)
mask = mask0+mask1

mask1=cv2.inRange(img_hsv,lower_red,upper_red)#HSV VAUES WILL BE DIFFERENT

mask=cv2.erode(mask,kernel,iterations=2)
mask=cv2.dilate(mask,kernel,iterations=2)
red_center=findcenter(mask)
targets.append(red_center)
cv2.imshow('redmask',mask)




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
lower_green=np.array([41,50,0])#HSV VAUES WILL BE DIFFERENT
upper_green=np.array([80,255,255])
greenmask=cv2.inRange(img_hsv,lower_green,upper_green)
greenmask=cv2.erode(greenmask,kernel,iterations=1)
greenmask=cv2.dilate(greenmask,kernel,iterations=1)
green_center=findcenter(greenmask)
targets.append(green_center)
cv2.imshow('g',greenmask)

#yellow
lower_yellow=np.array([18,150,64])#HSV VAUES WILL BE DIFFERENT
upper_yellow=np.array([40,255,255])
yellowmask=cv2.inRange(img_hsv,lower_yellow,upper_yellow)
yellowmask=cv2.erode(yellowmask,kernel,iterations=2)
yellowmask=cv2.dilate(yellowmask,kernel,iterations=2)
yellow_center=findcenter(yellowmask)
targets.append(yellow_center)
cv2.imshow('y',yellowmask)
print(targets)
gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
print(findfactor(gray))
#]cv2.imshow('res',res)
##    k=cv2.waitKey(5) & 0xFF
##    if k==ord('q') or k==27:
##        break




#red,blue,green,yellow
"""
Problems:
1.When  same colour balls are touching single object is taken
"""
import cv2
import numpy as np
img=cv2.imread('ball2.jpg',-1)
targets=[]
#gray=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
img_hsv=cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
grayscale=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
kernel=np.ones((5,5),np.uint8)
def dist(x1,y1,x2,y2):
    return ((x2-x1)**2+(y2-y1)**2)**(0.5)
def findfactor(grayimg):
    print(kernel)
    retval,threshold=cv2.threshold(grayimg,5,255,cv2.THRESH_BINARY_INV)#CHECK THRESHOLD VALUE
    #median=cv2.medianBlur(threshold,15)
    threshold=cv2.erode(threshold,kernel,iterations=3)
    #dilation=cv2.dilate(threshold,kernel,iterations=1)
    cv2.imshow('thresh',threshold)
    img_factor,contours_factor,hierarchy_factor=cv2.findContours(threshold,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    try:
        cv2.drawContours(img,contours_factor,-1,(255,0,0),3)#USE A BETTER ALGORITHM and remove this line img may not be defined
        cv2.imshow('img',img)
    except:
        pass
    squares=[]
    
    for i in contours_factor:
        approx = cv2.approxPolyDP(i,0.01*cv2.arcLength(i,True),True)
        #check if contour is square if dont use number of edges or use it after eroding and/or dilating to avoid partial square
        M_factor=cv2.moments(i)
        cx_factor = int(M_factor['m10']/M_factor['m00'])
        cy_factor = int(M_factor['m01']/M_factor['m00'])
        cv2.circle(img,(cx_factor,cy_factor), 5, (255,0,0), -1)   
        squares.append([cx_factor,cy_factor])

    if len(squares)==4:#EXCLUDING STARTING SQUARE
        distances=[dist(squares[0][0],squares[0][1],squares[1][0],squares[1][1]),dist(squares[0][0],squares[0][1],squares[2][0],squares[2][1]),dist(squares[0][0],squares[0][1],squares[3][0],squares[3][1])]
        diagonal=max(distances)
        sqnum=distances.index(diagonal)+1
        centerx=(squares[sqnum][0]+squares[0][0])/2
        centery=(squares[sqnum][1]+squares[0][1])/2
        factor=diagonal/127.26#length of diagonal in cm
        return factor,centerx,centery
    else:
        print(len(squares))
        print("Extra or less black squares found")#CHECK
#factor,centerx,centery=findfactor(grayscale)#grayscale image
def findcenter(colmask):
    funcvalue=[]
    img2,contours,hierarchy=cv2.findContours(colmask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    #cv2.drawContours(img,contours,-1,(203,192,203),3)
    if len(contours)>0:
        
        for i in contours:
            #print(cv2.contourArea(i))
            ((x, y), radius) = cv2.minEnclosingCircle(i)
            if radius>20:#CHANGE THIS VALUE ACCORDING TO CAMERA HEIGHT AND RADIUS
                
                approx = cv2.approxPolyDP(i,0.01*cv2.arcLength(i,True),True)
                
                if len(approx)>=7:#CHeck for diffent values higher than 5
                    
                    
                    M=cv2.moments(i)
                    
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])

                    cv2.circle(img,(cx,cy), 5, (0,0,0), -1)
                    funcvalue.append([cx,cy])
                    #Also check for area of moment afterwards
                else:
                    pass
            else:
                pass
    else:
        print("NO CONTOURS")
    return funcvalue    

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
redmask=cv2.erode(redmask,kernel,iterations=4)
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
lower_yellow=np.array([21,39,64])#HSV VAUES WILL BE DIFFERENT
upper_yellow=np.array([40,255,255])
yellowmask=cv2.inRange(img_hsv,lower_yellow,upper_yellow)
yellowmask=cv2.erode(yellowmask,kernel,iterations=2)
yellowmask=cv2.dilate(yellowmask,kernel,iterations=2)
yellow_center=findcenter(yellowmask)
targets.append(yellow_center)
cv2.imshow('y',yellowmask)

print(targets)
cv2.imshow('img',img)
cv2.waitKey(0)
cv2.destroyAllWindows()


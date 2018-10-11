import urllib.request
import cv2
import numpy as np

# Replace the URL with your own IPwebcam shot.jpg IP:port
url="http://169.254.1.1:8080/shot.jpg"
kernel=np.ones((5,5),np.uint8)
cap=cv2.VideoCapture(1)
while True:
    ret,img=cap.read()
    #imgResp = urllib.request.urlopen(url)
    #imgNp = np.array(bytearray(imgResp.read()),dtype=np.uint8)
    #img = cv2.imdecode(imgNp,-1)
    #return img

    # put the image on screen
    cv2.imshow("IPWebcam",img)
    grayimg=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    retval,threshold=cv2.threshold(grayimg,40,255,cv2.THRESH_BINARY_INV)#CHECK THRESHOLD VALUE
    median=cv2.medianBlur(threshold,15)
    threshold=cv2.erode(threshold,kernel,iterations=3)
    #dilation=cv2.dilate(threshold,kernel,iterations=1)
    
    cv2.imshow('thresh',threshold)
    img_factor,contours_factor,hierarchy_factor=cv2.findContours(threshold,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(img,contours_factor,-1,(255,0,0),3)
    cv2.imshow('contours',img)


# Program closes if q is pressed
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
cap.release()

import cv2
import numpy as np
import matplotlib.pyplot as plt
res=cv2.imread('ball.jpg',0)
mask=cv2.Canny(res,80,80)
kernel=np.ones((5,5),np.uint8)
'''erosion=cv2.erode(mask,kernel,iterations=1)
dilation=cv2.dilate(mask,kernel,iterations=1)
opening=cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernel)
closing=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,kernel)
'''
retval,thresh=cv2.threshold(res,230,255,cv2.THRESH_BINARY)
im2, contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)


# loop over the contours
for c in contours:
	# compute the center of the contour
    M = cv2.moments(c)
    if M['m00'] > 0:
          print('yo')
          cx = int(M['m10']/M['m00'])
          cy = int(M['m01']/M['m00'])
          cv2.circle(res, (cx, cy), 5, (0,0,255), -1)
cv2.imshow('th',thresh)
cv2.imshow('gray',res)

cv2.imshow('res',mask)
'''cv2.imshow('er',erosion)
cv2.imshow('di',dilation)
cv2.imshow('op',opening)
cv2.imshow('clo',closing)
'''
#cv2.imshow('smo',smoothed)
#cv2.imshow('blur',blur)
#cv2.imshow('median',median)
#cv2.imshow('bilateral',bilateral)
#cv2.imshow('gauss',gaus)
#cv2.imshow('otsu',otsu)
cv2.waitKey(0)
cv2.destroyAllWindows()


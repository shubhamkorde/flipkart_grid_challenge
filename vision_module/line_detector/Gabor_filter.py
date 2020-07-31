import numpy as np
import cv2
import matplotlib.pyplot as plt


ksize=10
sigma=3
theta=1*np.pi/2
lamda=1*np.pi/4
gamma=3.0
phi=0 


kernel = cv2.getGaborKernel((ksize,ksize),sigma,theta,lamda,gamma,phi,ktype=cv2.CV_32F)

img=cv2.imread('example_02.jpg')
img=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

fimg=cv2.filter2D(img,cv2.CV_8UC3,kernel)
kernel_resized=cv2.resize(kernel,(400,400))

#cv2.imshow('Original_Image',img)

#cv2.imshow('Filtered Image', fimg)
cv2.imshow('Kernel', kernel_resized)
cv2.waitKey(0)
cv2.destroyAllWindows()


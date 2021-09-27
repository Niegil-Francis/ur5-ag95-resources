import cv2
import numpy as np
import glob
import time

img_array = []
size = (30,30)
filenames=['data/images/fig_'+str(i+1)+'.png'   for i in range(len(glob.glob('data/images/*.png')))]
# print(filenames)
for filename in filenames:
    img = cv2.imread(filename)
    time.sleep(0.3)
    height, width, layers = img.shape
    size = (width,height)
    img_array.append(img)


out = cv2.VideoWriter('data/videos/project_new.avi',cv2.VideoWriter_fourcc(*'DIVX'), 15, size)
 
for i in range(len(img_array)):
    out.write(img_array[i])
    time.sleep(0.1)
out.release()
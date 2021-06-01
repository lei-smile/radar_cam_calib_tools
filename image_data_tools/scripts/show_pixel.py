#!/usr/bin/python
import matplotlib.pyplot as plt 
import matplotlib.image as mpimg
import numpy as np
import math

lena = mpimg.imread('./img.jpg')

print (lena.shape)
if (lena.shape[0] != 1080 or lena.shape[1] != 1920):
    print("resolution is not correct!!\n") 
else:
    plt.imshow(lena) 
    plt.axis('off')
    plt.show()

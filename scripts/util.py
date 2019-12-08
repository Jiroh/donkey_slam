import numpy as np
from PIL import Image

def split_image(img, viewing=False):
    img = Image.fromarray(img)
    w,h = img.size
    
    left_area = (0,0,w//2,h)
    right_area = (w//2,0,w,h)
    left_image = img.crop(left_area)
    right_image = img.crop(right_area)

    if(viewing):
        return left_image, right_image
    
    return np.array(left_image), np.array(right_image)
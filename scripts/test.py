import numpy as np
import cv2
from PIL import Image
import rospy

def split_image(img):
    
    img = Image.fromarray(img)
    w,h = img.size
    
    left_area = (0,0,w//2,h)
    right_area = (w//2,0,w,h)
    left_image = img.crop(left_area)
    right_image = img.crop(right_area)
    
    return left_image, right_image

if __name__ == "__main__":
    cap = cv2.VideoCapture(1)

    while(True):
        ret, frame = cap.read()
        l_img, r_img = split_image(frame)
        
        
        cv2.imshow('frame', np.array(l_img))
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
    cap.release()
    cv2.destroyAllWindows()
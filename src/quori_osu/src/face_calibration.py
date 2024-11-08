#!/usr/bin/env python3

import sys
import numpy as np
import cv2
from PIL import Image
import 


def move_selected_pixels():
    pass


if __name__ == "__main__":
    try:
        im = Image.open(sys.argv[1])
    except Exception as e:
        print(f"Usage: python3 {sys.argv[0]} <Image_Filename>")
        raise e


    print(np.array(im))
    cv2.imshow("window", np.array(im))
    cv2.waitKey(0)
    # gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    # _,binary = cv2.threshold(gray,100,255,cv2.THRESH_BINARY)
    # image, contours, h = cv2.findContours(binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    # cv2.drawContours(image,contours, -1, (0,255,0),3)
    
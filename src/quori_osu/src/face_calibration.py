#!/usr/bin/env python3
'''
Run this function to generate appropriately scaled faces for Quori. 
Pass in an unscaled face from base_faces and an optional yaml that tracks the change.
'''


import os, sys, yaml
import numpy as np
import cv2
from PIL import Image
from itertools import count


class Eye:
    """dataclass for representing a segment of the image."""
    def  __init__(self, circle, img_shape, max_tol=2, debug=True):
        circle = np.uint16(np.around(circle)) # round 
        self.center = np.array((circle[0], circle[1]))
        self.radius = circle[2]
        squaring = np.array([self.radius, self.radius])
        self.corners = [self.center - squaring, self.center+squaring]
        #make sure tol doesn't exceed size of img 
        min_index = min(self.corners[0])
        tol = min(min_index, max_tol)
        tol = min(tol, min(img_shape - self.corners[1] -1))
        self.corners[0] -= tol
        self.corners[1] += tol

        if debug:
            print("-----Eye Debug-----")
            print(f"Center: {self.center}")
            print(f"Radius: {self.radius}")
            print(f"Image Shape: {img_shape}")
            print(f"Box Size: {self.corners[1] - self.corners[0]}")
            print("-------------------")


def move_selected_pixels(image, bounding_box, dx, dy):
    """
    Move a selection of the image, doesn't handle edge cases
    """
    p1, p2 = bounding_box
    selection = image[p1[1]:p2[1], p1[0]:p2[0]].copy()
    image[p1[1]:p2[1], p1[0]:p2[0]] = np.mean(image)
    p1_prime = p1 + np.array((dx, dy))
    p2_prime = p2 + np.array((dx, dy))
    image[p1_prime[1]:p2_prime[1], p1_prime[0]:p2_prime[0]] = selection
    return p1_prime, p2_prime


def stretch_selected_pixels(image, bounding_box, sx, sy):
    """
    Scale a selection of the image, doesn't handle edge cases
    """
    p1, p2 = bounding_box
    selection = cv2.resize(image[p1[1]:p2[1], p1[0]:p2[0]], (0,0), fx=sx, fy=sy)
    out_shape = np.flip(selection.shape[0:2])
    image[p1[1]:p2[1], p1[0]:p2[0]] = np.mean(image)
    center = (p1 + p2) / 2
    p1_prime = np.uint16(center - out_shape // 2)
    p2_prime = np.uint16(center + (out_shape - out_shape // 2)) # handles when shape%2==1 
    image[p1_prime[1]:p2_prime[1], p1_prime[0]:p2_prime[0]] = selection
    return p1_prime, p2_prime


def find_eyes(image, min_radius=60):
    # need to specify min_radius because otherwise we find the inner circles too. And the thinking bubbles.
    im_rgb = image.convert('RGB')
    pixels = np.array(im_rgb).astype(np.uint8)
    gray = cv2.cvtColor(pixels, cv2.COLOR_RGB2GRAY)
    circles = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,1,20,
                            param1=50,param2=30,minRadius=min_radius,maxRadius=0)
 
    if len(circles[0]) != 2:
        print(f"Did not detect two eyes, instead found: {circles}")
        exit(1)
    
    # hold the eyes ready for processing
    eyes = [Eye(c, gray.shape) for c in circles[0]]
    eyes.sort(key=lambda eye: eye.center[0])
    return eyes


def getBackgroundColor(image, eyes):
    """gets the background color of the face."""
    im_rgb = image.convert('RGB')
    pixels = np.array(im_rgb).astype(np.uint8)
    gray = cv2.cvtColor(pixels, cv2.COLOR_RGB2GRAY)
    return np.mean(gray)


def shift_eyes(image, eyes, ops):
    """shifts the eyes based on the operations provided."""
    image = image.convert('RGB')
    pixels = np.array(image).astype(np.uint16)
    output = pixels.copy()
    for i in range(len(eyes)):
        op = ops[i]
        new_corners = move_selected_pixels(output, eyes[i].corners, dx=op['dx'], dy=op['dy'])
        new_corners = stretch_selected_pixels(output, new_corners, sx=op['sx'], sy=op['sy'])
    return Image.fromarray(np.uint8(output))


def zero_background(image, color):
    """zeros out the background of the image."""
    image = image.convert('RGB')
    pixels = np.array(image).astype(np.uint16)
    output = pixels.copy()
    gray = cv2.cvtColor(output, cv2.COLOR_RGB2GRAY)
    output[gray <= 50] = color
    return Image.fromarray(np.uint8(output))


if __name__ == "__main__":
    # default movements to run if yaml not provided
    operations = {
        0: { # First Eye
            'dx': 0,
            'dy': 0,
            'sx': 1,
            'sy': 1, 
        },
        1: { # Second Eye
            'dx': 0,
            'dy': 0,
            'sx': 1,
            'sy': 1, 
        }
    }
    try:
        filepath = sys.argv[1]
        im = Image.open(filepath)
        try:
            yaml_filepath = sys.argv[2]
            with open(yaml_filepath) as file:
                operations = yaml.safe_load(file)
        except IndexError:
            pass # use default dict
   
    except Exception as e:
        print(f"Usage: python3 {sys.argv[0]} <Image_Filename> [transform_list_yaml]")
        raise e

    # use hough to localize eyes in the image. Because we hold this constant, eyes cannot get larger during the gif
    eyes = find_eyes(im)
    color = getBackgroundColor(im, eyes)

    images = []
    delays = []
    try:
        for i in count(1):
            images.append(
                zero_background(shift_eyes(im, eyes, operations), color)
                )
            try:
                delays.append(im.info['duration'])
            except:
                delays.append(100)
            im.seek(i)
    except EOFError:
        pass

    filename = filepath.split(os.sep)[-1]
    images[0].save(os.path.join('quori_osu_supplemental/faces', filename), save_all=True, append_images=images[1:], duration=delays)
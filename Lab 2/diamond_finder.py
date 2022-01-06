#!/usr/bin/env
#Lab Partners: Manan Mayur Patel, Sukriti Singh
# import necessary packages
import cv2 as cv
import numpy as np
import os
from matplotlib import pyplot as plt
import time

#def finding_template(template_image, image, output_destination=os.path.abspath(os.getcwd())):
def finding_template(template_image, image, output_destination=None):

#    img = cv.imread('sample.jpg', 0)
    img = cv.cvtColor(image, cv.COLOR_BGR2GRAY) 
    
#    print(os.path.abspath(os.getcwd()))
    img2 = img.copy()

    # read the template file
    template = cv.imread(template_image,0)

    best_bounding_box = []
    # choose the template matching method
    meth = 'cv.TM_CCOEFF_NORMED'
    method = eval(meth)
    found = None
    best_loc = None
    
    #resizing the template to match the image
    for scale in np.linspace(10, 100, 10)[::-1]: 

            # resize the template according to the scale
            bounding_box =[]
            w = int(template.shape[1] * scale / 100)
            h = int(template.shape[0] * scale / 100)
            dim = (w, h)
            template_resized = cv.resize(template, dim, interpolation = cv.INTER_AREA)
            
            # if the resized image is larger than the image itself, then break (which will not actually happen)
            
            if template_resized.shape[0] > img.shape[0] or template_resized.shape[1] > img.shape[1]:
                break

            res = cv.matchTemplate(img,template_resized,method)
            threshold = 0.85
            loc = np.where( res >= threshold)
            
            #Make sure the same diamond is not counted more than once
            shape_count = 0
            mask = np.zeros(img.shape[:2], np.uint8)
            for pt in zip(*loc[::-1]):
                if mask[pt[1] + int(round(h/2)), pt[0] + int(round(w/2))] != 255:
                    mask[pt[1]:pt[1]+h, pt[0]:pt[0]+w] = 255
                    shape_count += 1
                    bounding_box.append([pt[0] + int(w/2), pt[1] + int(h/2)])

            #'found' holds the value of number of diamonds found
            #'best_bounding_boxes' has values of coordinates of diamonds (not redundant)
            if found is None or shape_count>found:
                found=shape_count
                best_loc=loc
                best_w = w
                best_h = h
                best_bounding_boxes=bounding_box

            if found==1:
                break
    
#    for pt in zip(*best_loc[::-1]):
#        cv.rectangle(img2, pt, (pt[0] + best_w, pt[1] + best_h), (255), 1)
        
#    if output_destination:
#       cv.imwrite(os.path.join(output_destination , 'sample' + str(time.time()) + '.jpg'), img2)
#	cv.imwrite(os.path.join(output_destination , 'sample' + '.jpg'), img2)
#    print(best_bounding_boxes)
    return(best_bounding_boxes)
 
def find_diamond():
    
    folder = 'input_imgs'
    destination_path = os.path.abspath(os.getcwd())+'/output_imgs'
    
    results = {}
    bounding_box = []
    try:
        os.mkdir(destination_path)
        
        # reading the folder for images
        for filename in os.listdir(folder):
            
            if '.jpg' in filename:
                bounding_box = finding_template('~/catkin_ws/src/walle_30_object_follower/scripts/template.jpg', filename, destination_path)
                results[filename]=bounding_box
        
    except FileExistsError:
        print ('Delete existing output_imgs folder')

    return results

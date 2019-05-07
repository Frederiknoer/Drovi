#!/usr/bin/env python3

import rasterio as ra
from rasterio.windows import Window
import numpy as np
from matplotlib import pyplot
import pyproj
import cv2

GlobalPumpkins = []

##################### Function define #############################################
def splitColoursHSV(input):
    org_img_hsv = cv2.cvtColor(input, cv2.COLOR_BGR2HSV)
    return cv2.split(org_img_hsv)

def splitColoursCieLab(input):
    org_img_lab = cv2.cvtColor(input, cv2.COLOR_BGR2LAB)
    return cv2.split(org_img_lab)

def countPumpkin(img):
    #Split color
    org_img_hue, org_img_sat, org_img_val = splitColoursHSV(img)
    org_img_l, org_img_a, org_img_b = splitColoursCieLab(img)
    #Apply threshold from mask in mandatory 1
    retval, thrs_a = cv2.threshold(org_img_a, 132, 255, cv2.THRESH_BINARY)
    retval, thrs_h = cv2.threshold(org_img_hue, 18, 255, cv2.THRESH_BINARY_INV)
    #Filter img
    combi_thresh = cv2.bitwise_and(thrs_a, thrs_h)
    morph_kernel = np.ones((3,3), np.uint8)
    combi_thresh = cv2.erode(combi_thresh, morph_kernel, iterations = 1)
    #Count connected components
    cc_seg_combi = cv2.connectedComponentsWithStats(combi_thresh, 4, cv2.CV_32S)
    cc_seg_combi_centroids = cc_seg_combi[3]
    #Draw
    print("Number of components using Hue(HSV) & A(CieLAB) segmentation (After morph filtering): %d" % len(cc_seg_combi_centroids))
    for px in cc_seg_combi_centroids:
        cv2.circle(img, (int(px[0]), int(px[1])), 7, (255, 0, 0))

    return img

##################### ODM Default #############################################
print("ODM Default: ")
it = 1
w = 500
h = 500
Col_off = 0
Row_off = 4250 # h*5 # 0 for zero offset

ODM = '../tiffExample/default.tif'
window = Window(Col_off,Row_off,w,h)

for i in range(it):
    Col_off = 900
    for j in range(it):
        window = Window(Col_off,Row_off,w,h) # Create tile in window size | Credit to https://geohackweek.github.io/raster/04-workingwithrasters/

        with ra.open(ODM, 'r') as src:
            # print(src.profile)
            # print(src.colorinterp) # Shows that the chanels, are R,G,B,A
            r,b,g,a = src.read(window=window) # Split into the 4 challenges
            subset = cv2.merge((g,b,r)) # Merge into a h*w*channels array, discarding aplhpa. For some reason Green,Blue,Red seems to be the right format.
        #test = countPumpkin(subset)
        print("| Row , Col |\n" , Row_off , Col_off )
        cv2.imshow("img",subset)
        cv2.waitKey()
        cv2.destroyAllWindows()
        cv2.imwrite('../output/ODMdefault.png',subset)



        Col_off += w
    Row_off += h

##################### ODM Default #############################################
print("ODM Fast: ")
it = 1
w = 500
h = 500
Col_off = 0
Row_off = 4275 # h*5 # 0 for zero offset

ODM = '../tiffExample/fast.tif'
window = Window(Col_off,Row_off,w,h)

for i in range(it):
    Col_off = 925
    for j in range(it):
        window = Window(Col_off,Row_off,w,h) # Create tile in window size | Credit to https://geohackweek.github.io/raster/04-workingwithrasters/

        with ra.open(ODM, 'r') as src:
            # print(src.profile)
            # print(src.colorinterp) # Shows that the chanels, are R,G,B,A
            r,b,g,a = src.read(window=window) # Split into the 4 challenges
            subset = cv2.merge((g,b,r)) # Merge into a h*w*channels array, discarding aplhpa. For some reason Green,Blue,Red seems to be the right format.
        #test = countPumpkin(subset)
        cv2.imshow("img",subset)
        cv2.waitKey()
        cv2.destroyAllWindows()
        cv2.imwrite('../output/ODMfast.png',subset)
        print("| Row , Col |\n" , Row_off , Col_off )

        Col_off += w
    Row_off += h

#####################  Metashape #############################################
print("Metashape: ")
it = 1
w = 500
h = 500
Col_off = 0
Row_off = 4275*2.4 + 300    # h*5 # 0 for zero offset

ODM = '../tiffExample/Metashape-default.tif'
window = Window(Col_off,Row_off,w,h)

for i in range(it):
    Col_off = 925*3.2+250
    for j in range(it):
        window = Window(Col_off,Row_off,w,h) # Create tile in window size | Credit to https://geohackweek.github.io/raster/04-workingwithrasters/

        with ra.open(ODM, 'r') as src:
            # print(src.profile)
            # print(src.colorinterp) # Shows that the chanels, are R,G,B,A
            r,b,g,a = src.read(window=window) # Split into the 4 challenges
            subset = cv2.merge((g,b,r)) # Merge into a h*w*channels array, discarding aplhpa. For some reason Green,Blue,Red seems to be the right format.
        #test = countPumpkin(subset)
        cv2.imshow("img",subset)
        cv2.waitKey()
        cv2.destroyAllWindows()
        cv2.imwrite('../output/metashape.png',subset)

        print("| Row , Col |\n" , Row_off , Col_off )

        Col_off += w
    Row_off += h

##################### MME #############################################
print("MME: ")
it = 1
w = 500
h = 500
Col_off = 500+75
Row_off = 9000 # h*5 # 0 for zero offset

ODM = '../tiffExample/MapsMadeEasy.tif'
window = Window(Col_off,Row_off,w,h)

for i in range(it):
    Col_off = 1000
    for j in range(it):
        window = Window(Col_off,Row_off,w,h) # Create tile in window size | Credit to https://geohackweek.github.io/raster/04-workingwithrasters/

        with ra.open(ODM, 'r') as src:
            # print(src.profile)
            # print(src.colorinterp) # Shows that the chanels, are R,G,B,A
            r,b,g,a = src.read(window=window) # Split into the 4 challenges
            subset = cv2.merge((g,b,r)) # Merge into a h*w*channels array, discarding aplhpa. For some reason Green,Blue,Red seems to be the right format.
        #test = countPumpkin(subset)
        cv2.imshow("img",subset)
        cv2.waitKey()
        cv2.destroyAllWindows()
        cv2.imwrite('../output/MME.png',subset)
        print("| Row , Col |\n" , Row_off , Col_off )

        Col_off += w
    Row_off += h


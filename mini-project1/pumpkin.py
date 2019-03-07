import numpy as np
import cv2
import math
import os
import sys

if sys.version_info[0] < 3:
    user_input_1 = raw_input("Input file name of image(without /folder, with filtype): ") #python 2
else:
    user_input_1 = input("Input file name of image(without /folder, with filtype): ") #python3

if sys.version_info[0] < 3:
    user_input_2 = raw_input("Input file name of image_MASK(without /folder, with filtype): ") #python 2
else:
    user_input_2 = input("Input file name of image_MASK(without /folder, with filtype): ") #python3
image_name = "input/" + user_input_1
image_mask_name = "input/" + user_input_2


def splitColoursBGR(input):
    return input[:,:,0], input[:,:,1], input[:,:,2]

def splitColoursHSV(input):
    org_img_hsv = cv2.cvtColor(input, cv2.COLOR_BGR2HSV)
    return cv2.split(org_img_hsv)

def splitColoursCieLab(input):
    org_img_lab = cv2.cvtColor(input, cv2.COLOR_BGR2LAB)
    return cv2.split(org_img_lab)

def colourStats(input, space):
    if space == "BGR":
        a, b, c = splitColoursBGR(input)
    if space == "HSV":
        a, b, c = splitColoursHSV(input)
    if space == "CieLAB":
        a, b, c = splitColoursCieLab(input)

    mean_a = np.mean(np.ma.masked_equal(a,0))
    mean_b = np.mean(np.ma.masked_equal(b,0))
    mean_c = np.mean(np.ma.masked_equal(c,0))
    std_a = np.std(np.ma.masked_equal(a,0))
    std_b = np.std(np.ma.masked_equal(b,0))
    std_c = np.std(np.ma.masked_equal(c,0))

    if space == "BGR":
        print("Mean of Blue (BGR): %d" % mean_a)
        print("Mean of Green (BGR): %d" % mean_b)
        print("Mean of Red (BGR): %d" % mean_c)
        print("Std. Deviation of Blue (RGB): %.3f" % std_a)
        print("Std. Deviation of Green (RGB): %.3f" % std_b)
        print("Std. Deviation of Red (RGB): %.3f" % std_c)
    if space == "HSV":
        print("Mean of Hue (HSV): %d" % mean_a)
        print("Mean of Saturation (HSV): %d" % mean_b)
        print("Mean of Value (HSV): %d" % mean_c)
        print("Std. Deviation of Hue (HSV): %.3f" % std_a)
        print("Std. Deviation of Saturation (HSV): %.3f" % std_b)
        print("Std. Deviation of Value (HSV): %.3f" % std_c)
    if space == "CieLAB":
        print("Mean of L (CieLAB): %.3f" % mean_a)
        print("Mean of A (CieLAB): %.3f" % mean_b)
        print("Mean of B (CieLAB): %.3f" % mean_c)
        print("Std. Deviation of L (CieLab): %.3f" % std_a)
        print("Std. Deviation of A (CieLab): %.3f" % std_b)
        print("Std. Deviation of B (CieLab): %.3f" % std_c)
    return mean_a, mean_b, mean_c

def vis4imgs(tl, tr, bl, br, window_name, show_save):
    if(len(tl.shape) < 3):
        tl = cv2.cvtColor(tr, cv2.COLOR_GRAY2BGR)
    if(len(tr.shape) < 3):
        tr = cv2.cvtColor(tr, cv2.COLOR_GRAY2BGR)
    if(len(bl.shape) < 3):
        bl = cv2.cvtColor(bl, cv2.COLOR_GRAY2BGR)
    if(len(br.shape) < 3):
        br = cv2.cvtColor(br, cv2.COLOR_GRAY2BGR)

    img_top_row = np.hstack((tl, tr))
    img_bot_row = np.hstack((bl, br))
    vis_img = np.vstack((img_top_row, img_bot_row))

    if(show_save == 'show'):
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 1280, 720)
        cv2.imshow(window_name, vis_img)
    elif (show_save == 'save'):
        savestring = "output/" + window_name + ".png"
        cv2.imwrite(savestring, vis_img)



#Main:
os.mkdir("output")
#LOAD IMAGES:
input_img = cv2.imread(image_name, 1)
img_masked = cv2.imread(image_mask_name, 1) # < --- EXERCISE 1
org_img = input_img.copy()

#SPLIT COLOUR SPACES
org_img_blue, org_img_green, org_img_red = splitColoursBGR(input_img)
org_img_hue, org_img_sat, org_img_val = splitColoursHSV(input_img)
org_img_l, org_img_a, org_img_b = splitColoursCieLab(input_img)

#USE THE MASK
mask = cv2.inRange(src=img_masked,
                    lowerb=np.array([255, 255, 255]),
                    upperb=np.array([255, 255, 255]))

img_pumpkin = cv2.bitwise_and(input_img, img_masked)
print("EXERCISE 1: Image saved in output folder \n")
vis4imgs(org_img, img_masked, mask, img_pumpkin, "EX1_mask", 'save')


#GET COLOUR STATS FOR THE ORG IMAGE USING THE MASK (EXERCISE 2):
print("EXERCISE 2:")
mean_blue, mean_green, mean_red = colourStats(img_pumpkin, "BGR")
colourStats(img_pumpkin, "HSV")
colourStats(img_pumpkin, "CieLAB")
print("\n")


#THRESHOLDING ALL THE COLOUR SPACES INDDUVIDUALLY:
retval, thrs_red = cv2.threshold(org_img_red, 160, 255, cv2.THRESH_BINARY)
retval, thrs_green = cv2.threshold(org_img_green, 83, 255, cv2.THRESH_BINARY)
retval, thrs_blue = cv2.threshold(org_img_blue, 25, 255, cv2.THRESH_BINARY)
retval, thrs_blue_inv = cv2.threshold(org_img_blue, 100, 255, cv2.THRESH_BINARY)

retval, thrs_h = cv2.threshold(org_img_hue, 18, 255, cv2.THRESH_BINARY_INV)
retval, thrs_s = cv2.threshold(org_img_sat, 214, 255, cv2.THRESH_BINARY)
retval, thrs_v = cv2.threshold(org_img_val, 137, 255, cv2.THRESH_BINARY)

retval, thrs_l = cv2.threshold(org_img_l, 102, 255, cv2.THRESH_BINARY)
retval, thrs_a = cv2.threshold(org_img_a, 132, 255, cv2.THRESH_BINARY)
retval, thrs_b = cv2.threshold(org_img_b, 132, 255, cv2.THRESH_BINARY)

#EXERCISE 2.1 RGB SEGMENT:
seg_rgb = cv2.bitwise_and(thrs_red, thrs_blue_inv)
cc_seg_rgb = cv2.connectedComponentsWithStats(seg_rgb, 4, cv2.CV_32S)
cc_seg_rgb_centroids = cc_seg_rgb[3]
print("EX(2.1): Number of components using RGB segmentation: %d" % len(cc_seg_rgb_centroids))
cv2.imwrite("output/EX2_1_RGB_SEGEMENT.png", seg_rgb)

#EXERCISE 2.2 CieLAB SEGMENT:
cc_seg_lab = cv2.connectedComponentsWithStats(thrs_a, 4, cv2.CV_32S)
cc_seg_lab_centroids = cc_seg_lab[3]
print("EX(2.2): Number of components using CieLab segmentation: %d" % len(cc_seg_lab_centroids))
cv2.imwrite("output/EX2_2_CieLAB_SEGEMENT.png", thrs_a)

#Exercise 2.3 Histogram backprojection
img_pumpkin_hsv = cv2.cvtColor(img_pumpkin,cv2.COLOR_BGR2HSV)
img_masked_gray = cv2.cvtColor(img_masked, cv2.COLOR_BGR2GRAY)
org_img_hsv = cv2.cvtColor(org_img, cv2.COLOR_BGR2HSV)

# Normalize histogram
img_pumpkin_his = cv2.calcHist(images=[img_pumpkin_hsv],channels=[0, 1],mask=mask,histSize=[180, 256],ranges=[0, 180, 0, 256])
cv2.normalize(src=img_pumpkin_his,dst=img_pumpkin_his, alpha=0, beta=255,norm_type=cv2.NORM_MINMAX)
# Backproject
backproject_image = cv2.calcBackProject(images=[org_img_hsv],channels=[0, 1],hist=img_pumpkin_his,ranges=[0, 180, 0, 250],scale=1)
# Closing
kernel = np.ones((5,5),np.uint8)
backproject_image_closed = cv2.morphologyEx(backproject_image, cv2.MORPH_CLOSE, kernel)
#Count
cc_seg_hb = cv2.connectedComponentsWithStats(backproject_image_closed, 8, cv2.CV_32S)
cc_seg_hb_centroids = cc_seg_hb[3]
print("EX(2.3): Number of components using Histogram backprojection: %d" % len(cc_seg_hb_centroids))
cv2.imwrite("output/EX2_3_HIST_Backprojection.png", backproject_image)

#Exercise 2.4 Distance in RGB space:
print("\nEXERCISE 2.4: Distance in RGB space started ...")

pumpkincolor = np.array([mean_blue,mean_green,mean_red])

img_gray = cv2.cvtColor(input_img,cv2.COLOR_BGR2GRAY)

img_rgb_copy = input_img.reshape(np.size(input_img[:,:,0]),3)
img_gray_copy = img_gray.reshape(np.size(input_img[:,:,0]))
img_masked_copy = img_masked.reshape(np.size(img_masked[:,:,0]),3)

readImage = False            # Set to False to Perform
if readImage:
    black = cv2.imread("Output/black80.JPG",0)
else:
    black = np.zeros(np.shape(img_gray_copy))
    for idx, pix in enumerate(img_rgb_copy):
        distance = np.sqrt(np.square(pix[0] - pumpkincolor[0]) + (np.square(pix[1] - pumpkincolor[1])) + (np.square(pix[2]-pumpkincolor[2])))
        if distance < 80:
            black[idx] = 255
    black = black.reshape(np.shape(img_gray))


morph_kernel = np.ones((7,7),np.uint8)
cv2.imwrite("output/EX2_4_RGB_space.png", black)
img_a_filtered = cv2.morphologyEx(black, cv2.MORPH_OPEN, morph_kernel)

img_a_filtered = (img_a_filtered).astype('uint8')
output = cv2.connectedComponentsWithStats(img_a_filtered, 4, cv2.CV_32S)
centroids = output[3]
print("\nEX(2.4): Number of components using RGB segmentation: %d" % len(centroids))
cv2.imwrite("output/EX2_4_RGB_space_MORPH.png", img_a_filtered)

#Exercise 3:
print("\nEXERCISE 3: ")
combi_thresh = cv2.bitwise_and(thrs_a, thrs_h)
cc_seg_combi = cv2.connectedComponentsWithStats(combi_thresh, 4, cv2.CV_32S)
cc_seg_combi_centroids = cc_seg_combi[3]
print("Number of components using Hue(HSV) & A(CieLAB) segmentation: %d" % len(cc_seg_combi_centroids))
cv2.imwrite("output/EX3_HUE_CieLAB.png", combi_thresh)

#Exercise 4:
morph_kernel = np.ones((3,3), np.uint8)
combi_thresh = cv2.erode(combi_thresh, morph_kernel, iterations = 1)
cv2.imwrite("output/EX4_HUE_CieLAB.png", combi_thresh)


#Exercise 5:
print("\nEXERCISE 5: ")
cc_seg_combi = cv2.connectedComponentsWithStats(combi_thresh, 4, cv2.CV_32S)
cc_seg_combi_centroids = cc_seg_combi[3]
print("Number of components using Hue(HSV) & A(CieLAB) segmentation (After morph filtering): %d" % len(cc_seg_combi_centroids))

#Exercise 6:


#Exercise 7:
for px in cc_seg_combi_centroids:
    cv2.circle(org_img, (int(px[0]), int(px[1])), 15, (255, 0, 0))
cv2.imwrite("output/EX7_circle.png", org_img)

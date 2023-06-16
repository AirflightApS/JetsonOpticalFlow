import numpy as np
import cv2

from picamera2.picamera2 import *
from time import sleep

camera = Picamera2()
camera.start()
image = camera.capture_image()
np_array = camera.capture_array()
print("captured")
cv2.imshow("camera", np_array)
# Read the query image as query_img
# and train image This query image
# is what you need to find in train image
# Save it in the same directory
# with the name image.jpg  
query_img = cv2.imread('query.jpg')
train_img = cv2.imread('train.jpg')
dimensions_trainheight = train_img.shape[0]
dimensions_queryheight = query_img.shape[0]
scale = dimensions_trainheight/dimensions_queryheight
print(scale)
width = int(query_img.shape[1] * scale)
height = int(query_img.shape[0] * scale)
dim = (width, height)

query2 = cv2.resize(query_img, dim, interpolation = cv2.INTER_CUBIC)
print(query2.shape)
print(train_img.shape)
# Convert it to grayscale
query_img_bw = cv2.cvtColor(query2,cv2.COLOR_BGR2GRAY)
train_img_bw = cv2.cvtColor(train_img, cv2.COLOR_BGR2GRAY)
   
# Initialize the ORB detector algorithm
orb = cv2.ORB_create()
   
# Now detect the keypoints and compute
# the descriptors for the query image
# and train image
queryKeypoints, queryDescriptors = orb.detectAndCompute(query_img_bw,None)
trainKeypoints, trainDescriptors = orb.detectAndCompute(train_img_bw,None)
  
# Initialize the Matcher for matching
# the keypoints and then match the
# keypoints
matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
matches = matcher.match(queryDescriptors,trainDescriptors)
print(len(matches))
# draw the matches to the final image
# containing both the images the drawMatches()
# function takes both images and keypoints
# and outputs the matched query image with
# its train image
final_img = cv2.drawMatches(query2, queryKeypoints, 
train_img, trainKeypoints, matches[:],None)
   
final_img = cv2.resize(final_img, (1000,650))
  
# Show the final image
cv2.imshow("Matches", final_img)
cv2.waitKey(3000)
# query for qr code
query_img = cv2.imread('tokyo.jpg')
query_img_bw = cv2.cvtColor(query_img,cv2.COLOR_BGR2GRAY)
queryKeypoints, queryDescriptors = orb.detectAndCompute(query_img_bw,None)

while True:
    np_array = camera.capture_array() # capture image
    train_img_bw = cv2.cvtColor(np_array, cv2.COLOR_BGR2GRAY)
    # get points 
    trainKeypoints, trainDescriptors = orb.detectAndCompute(train_img_bw,None)
    matches = matcher.match(queryDescriptors,trainDescriptors)
    # Sort them in the order of their distance.
    matches = sorted(matches, key = lambda x:x.distance)

    print(len(matches))
    # draw the matches to the final image
    # containing both the images the drawMatches()
    # function takes both images and keypoints
    # and outputs the matched query image with
    # its train image
    final_img = cv2.drawMatches(query_img, queryKeypoints, 
    np_array, trainKeypoints, matches[:15],None)
       
    final_img = cv2.resize(final_img, (1000,650))
      
    # Show the final image
    cv2.imshow("Matchesr", final_img)
    cv2.waitKey(10)

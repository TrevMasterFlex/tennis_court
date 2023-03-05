conda create -n tennis_court python=3.10<br/>
conda activate tennis_court<br/>
conda install -y jupyter matplotlib opencv<br/>
jupyter notebook<br/><br/>


Solution:

Step #1) - Detect Lines Within Image:
- Convert each image from rgb color space to grayscale
- Produce binary image from running a canny edge detector on this grayscale image
- Dilate the binary image
- Run a hough transform to detect finite line segments on the dilated binary image

Step #2) - Find the Homography:
- Seven points are preferred in order to detect the homography of interest (for accuracy). These are the line intersections found at either end of a tennis court.
- The distance is calculated between an end point of a line segment and another line for every line segment pair. End points that fall below a very low distance threshold are collected. This results in all 7 T-shape and corners being detected on the tennis court.
- Using the 7 points detected and the standard locations of where to find these points on a tennis court, a homography is generated. A perspective tranform is generated from the homography. The standard locations used to find the homography are tranformed with this perspective transform. Error is calculated off the difference between these transformed points and those detected. All other standard tennis court line intersection points are transformed with this perspective transform and used to draw sections of the tennis court.

![alt text](https://github.com/TrevMasterFlex/tennis_court/blob/main/output0.png?raw=true)
![alt text](https://github.com/TrevMasterFlex/tennis_court/blob/main/output1.png?raw=true)

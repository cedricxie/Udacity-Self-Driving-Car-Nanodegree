# **Finding Lane Lines on the Road** 

The goals / steps of this project are the following:
* Make a pipeline that finds lane lines on the road
* Reflect on your work in a written report


[//]: # (Image References)

[image1]: ./test_images_output/solidWhiteCurve_gray.jpg "Grayscale"
[image2]: ./test_images_output/solidWhiteCurve_blur_gray.jpg "Grayscale+GaussianBlur"
[image3]: ./test_images_output/solidWhiteCurve_edges.jpg "Grayscale+GaussianBlur+Canny"
[image4]: ./test_images_output/solidWhiteCurve_hough.jpg "Grayscale+GaussianBlur+Canny+Hough"
[image5]: ./test_images_output/solidWhiteCurve_final.jpg "Grayscale+GaussianBlur+Canny+Hough+Overlap"

---

### Reflection

### 1. Pipeline Description

My pipeline consisted of 5 steps. 

1. Grayscale: I converted the images to grayscale using "cv2.cvtColor".
![alt text][image1]
2. Guassian smoothing: The "cv2.GaussianBlur" is applied on the image.
![alt text][image2]
3. Edge detection: The "cv2.Canny" is applied to obtain gradient plot.
![alt text][image3]
4. Hough transform: A mask with polygon region of interest is first created then together with "cv2.HoughLinesP" the lane lines are obtained.  
![alt text][image4]
5. Overlay plots: A final figure is created with "cv2.addWeighted".
![alt text][image5]


To draw a single line on the left and right lanes, I modified the function by:

1. Instead of draw every lines, an average value is calculated for the slope, the x coordiate, and the y coordinate.

2. A final line is drawn using the following function.
\begin{equation}
y-y_{ave}=k_{ave}*{x-x_{ave}}
\end{equation}


### 2. Identify potential shortcomings with your current pipeline


One potential shortcoming would be the line drawn is not stable, it is shaking frequently. 

Another shortcoming could be not able to deal with curved lane, as in the "Challenge" part.


### 3. Suggest possible improvements to your pipeline

A possible improvement would be to add a filter in the calculatio of the slope.(only allow slopes in a certain range to be considered.)

Another potential improvement could be that, instead of a single line, draw a curved line. (Or multiple small lines connected together? Not sure yet.)

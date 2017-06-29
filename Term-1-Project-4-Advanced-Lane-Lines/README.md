# **Advanced Lane Finding Project**

The goals / steps of this project are the following:

* Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
* Apply a distortion correction to raw images.
* Use color transforms, gradients, etc., to create a thresholded binary image.
* Apply a perspective transform to rectify binary image ("birds-eye view").
* Detect lane pixels and fit to find the lane boundary.
* Determine the curvature of the lane and vehicle position with respect to center.
* Warp the detected lane boundaries back onto the original image.
* Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.

[//]: # (Image References)

[image1]: ./camera_cal/calibration2.jpg "Original Distorted Image"
[image2]: ./output_images/calibration_found_12.jpg "Original Distorted Image with Corners Found"
[image3]: ./output_images/results_undistortion.jpg "Comparison before and after Undistortion"
[image4]: ./output_images/results_binaries.jpg "Effect of Gradient, Color Channel Operations"
[image5]: ./output_images/results_video_demo.jpg "Demo Output"
[image6]: ./output_images/results_warped.jpg "Warped Image"
[image7]: ./output_images/results_fitting.jpg "Searching and Fitting Results"
[video1]: ./output_images/test_output.mp4 "Video Output"

---

### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.

You're reading it!

### Camera Calibration

#### 1. Briefly state how you computed the camera matrix and distortion coefficients. 

The code for this step is contained in the first and second code cell of the IPython notebook located [here](https://github.com/cedricxie/CarND-Advanced-Lane-Lines/blob/master/advanced_lane_lines_YX.ipynb).  

Two lists are critical to camera calibration: the `objpoints` and `imgpoints`.
1. `objpoints` is a list of the (x, y, z) coordinates of the chessboard corners in the real world. It is assumed that chessboard is fixed on the (x, y) plane at z=0
2. `imgpoints` will be appended with the (x, y) pixel position of each of the corners in the image plane with each successful chessboard detection.  

An example of original image and the results after finding and drawing chessboard corners is shown below.

![Original Distorted Image][image1]
![Original Distorted Image with Corners Found][image2]

Then, the function `cv2.calibrateCamera()` is implemented to find the **camera matrix** and **distortion coefficient**. They are further applied to test image using the `cv2.undistort()` function and obtained this result: 

![Comparison before and after Undistortion][image3]

### Pipeline (single images)

#### 1. Provide an example of a distortion-corrected image.

An example of applying distortion correction to one of the test images is shown in the section above.

#### 2. Describe how (and identify where in your code) you used color transforms, gradients or other methods to create a thresholded binary image.

I used a combination of color and gradient thresholds to generate a binary image (detailed codes are in the third cell of the [notebook](https://github.com/cedricxie/CarND-Advanced-Lane-Lines/blob/master/advanced_lane_lines_YX.ipynb)). Here's an example of my output for this step.

![Effect of Gradient, Color Channel Operations][image4]

As demonstrated, the filters I applied include:
1. Gradient in the X direction with `cv2.Sobel` function
2. Magnitude of the Gradient in X and Y direction
3. Directional Gradient
4. Combined Gradient of 1-3 above
5. Saturation Channel of image in HLS format
6. Saturation Channel combined with Gradient in the X Direction

#### 3. Describe how (and identify where in your code) you performed a perspective transform.

A function `perspective_transform` is created in the third cell of the [notebook](https://github.com/cedricxie/CarND-Advanced-Lane-Lines/blob/master/advanced_lane_lines_YX.ipynb). This function is capable of both warping and unwarping an image, depending on the value of the parameter `flag`.

1. When warping an image (`flag` set to be `1`), the image is first undistorted with `cv2.undistort`, then the perspective transform `M` is calculated using `cv2.getPerspectiveTransform` with source points `src` and destination points `dst`. In the end, the warped image is obtained using function `cv2.warpPerspective`. The source and destination points are listed as follows.

| Source        | Destination   | 
|:-------------:|:-------------:| 
| 260, 680      | 300, 700      | 
| 1050, 680     | 900, 700      |
| 590, 450      | 300, 50       |
| 680, 450      | 900, 50       |

I verified that my perspective transform was working as expected by drawing the `src` and `dst` points onto a test image and its warped counterpart to verify that the lines appear parallel in the warped image.

![Warped Image][image6]

#### 4. Describe how (and identify where in your code) you identified lane-line pixels and fit their positions with a polynomial?

In the fourth cell of [notebook](https://github.com/cedricxie/CarND-Advanced-Lane-Lines/blob/master/advanced_lane_lines_YX.ipynb), I applied the sliding window method together with a convolution on my warped image to find the best window center positions. In the following image, the red and blue boxes show the centers found on the left and right lanes perspectively. Then I fit my lane lines with a 2nd order polynomial as the yellow lines.

![Searching and Fitting Results][image7]

#### 5. Describe how (and identify where in your code) you calculated the radius of curvature of the lane and the position of the vehicle with respect to center.

Similarly fourth cell of [notebook](https://github.com/cedricxie/CarND-Advanced-Lane-Lines/blob/master/advanced_lane_lines_YX.ipynb), for a function as $f(y) = Ay^2 + By +C$, the curvature is calculated as follows.

$R_curvature = \frac{{{{\left( {1 + {{(2Ay + B)}^2}} \right)}^{3/2}}}}{{\left| {2A} \right|}}$

#### 6. Provide an example image of your result plotted back down onto the road such that the lane area is identified clearly.

All the used functions and supporting codes are put into the fifth cell of the [notebook](https://github.com/cedricxie/CarND-Advanced-Lane-Lines/blob/master/advanced_lane_lines_YX.ipynb). Here is an example of my result on a test image:

![Demo Output][image5]

---

### Pipeline (video)

#### 1. Provide a link to your final video output.

Here's a [link to my video result](https://youtu.be/WlNl_NjvsrA)

---

### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

The major problem I faced in the project is combining all the available codes together. And due to the time issue, I haven't set up the more sophisticated code structure as suggested.
The code might fail when the curvature is really sharp, and could potentially be solved by increasing the resolution of the windows search.

# **Vehicle Detection Project**

The goals / steps of this project are the following:

* Perform a Histogram of Oriented Gradients (HOG) feature extraction on a labeled training set of images and train a classifier Linear SVM classifier
* Optionally, you can also apply a color transform and append binned color features, as well as histograms of color, to your HOG feature vector. 
* Note: for those first two steps don't forget to normalize your features and randomize a selection for training and testing.
* Implement a sliding-window technique and use your trained classifier to search for vehicles in images.
* Run your pipeline on a video stream (start with the test_video.mp4 and later implement on full project_video.mp4) and create a heat map of recurring detections frame by frame to reject outliers and follow detected vehicles.
* Estimate a bounding box for vehicles detected.

[//]: # (Image References)
[image1]: ./output_images/image1.png
[image2]: ./output_images/image2.png
[image3]: ./output_images/image9.png
[image4]: ./output_images/image10.png
[image5]: ./output_images/image11.png
[image6]: ./output_images/image12.png
[image7]: ./output_images/clip1.JPG
[image8]: ./output_images/clip2.JPG
[image9]: ./output_images/clip3.JPG
[image10]: ./output_images/clip4.JPG
[image11]: ./output_images/image15.png
[image12]: ./output_images/image16.png
[video1]: ./output_images/test_output.mp4

---

### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.

You're reading it!

### Histogram of Oriented Gradients (HOG)

#### 1. Explain how (and identify where in your code) you extracted HOG features from the training images.

The code for the function of extracting HOG features is contained in the 3rd code cell of the [Jupyter notebook](https://github.com/cedricxie/CarND-Vehicle-Detection/blob/master/vehicle_detection_YX.ipynb)

I started by reading in all the `vehicle` and `non-vehicle` images.  Here is an example of one of each of the `vehicle` and `non-vehicle` classes in both 64 by 64 original shape and 16 by 16 reduced size. As you can see, even in the image of reduced size, the difference between `vehicle` and `non-vehicle` images are still distinguishable.

![Example Images of 64 by 64][image1]
![Example Images of 16 by 16][image2]

Furthermore, the HOG images for both images in gray scale are shown here in order to get a feel of how they look like.

![Example Car Images and HOG Visualization][image3]
![Example Non-Car Images and HOG Visualization][image4]

I then displayed them using 3D plots in different color spaces. Here is an example using the `RGB`, `HSV`, `YCrCb` color spaces. As shown, the images in the `YCrCb` color space seems to be the most clear and organized among all.

![Vehicle Image][image7]
![Non-Vehicle Image][image8]

#### 2. Explain how you settled on your final choice of HOG parameters.

I tried 3 different combinations of parameters, namely:

| Trial  | Color Space   | Features Type   | 
|:------:|:-------------:|:---------------:| 
| 1      | HSV           |  HOG Only       | 
| 2      | YCrCb         | Colors + HOG    | 
| 3      | YCrCb         | HOG Only        | 

1. For the 1st trial, the choice of parameters are as follows:

| Parameters       | Values     |
|:----------------:|:----------:|
| Orient           | 9          |
| Pixel per Cell   | 8          |
| Cell per Block   | 2          | 

The feature size therefore is: 5292 = 9 * 2 * 2 * 7 * 7 * 3 

2. For the 2nd trial, the choice of parameters are:

| Parameters        | Values     |
|:-----------------:|:----------:|
| Orient            | 11         |
| Pixel per Cell    | 16         |
| Cell per Block    | 2          | 
| Reduced Image Size| 16 x 16    | 
| Number of Bins    | 32         | 

The feature size is: 2052 = 11 * 2 * 2 * 3 * 3 * 3 + 256 * 3 + 32  3

3. For the 3rd trial, the choice of parameters are:

| Parameters        | Values     |
|:-----------------:|:----------:|
| Orient            | 11         |
| Pixel per Cell    | 16         |
| Cell per Block    | 2          |  

The feature size is: 1188 = 11 * 2 * 2 * 3 * 3 * 3

Two test cases are evaluated for all three trials to compare the performance. As demonstrated, HSV color space with HOG features tends to over predict cars, and the YCrCb color space with Color and HOG features tends to under predict the cars. Among all, the 3rd trial, YCrCb performs the best, and is therefore chosen.

![Test Case 1][image9]
![Test Case 2][image10]

The feature vectors for both Car and Non-Car images are plotted for visualization.

![Vehicle Image and Its Feature Vector][image5]
![Non-Vehicle Image and Its Feature Vector][image6]

#### 3. Describe how (and identify where in your code) you trained a classifier using your selected HOG features (and color features if you used them).

I trained a linear SVM, which is defined as ```clf = LinearSVC()```, as in the 6th Code Cell in the [Jupyter Notebook](https://github.com/cedricxie/CarND-Vehicle-Detection/blob/master/vehicle_detection_YX.ipynb). It is fitted to the training data, and validated by the testing data, which are created by ```train_test_split``` tool in ```sklearn.model_selection```.

### Sliding Window Search

#### 1. Describe how (and identify where in your code) you implemented a sliding window search.

The Sliding window search is defined as in the 8th Code Cell in the [Jupyter Notebook]().
1. The top and bottom position of the searched window is defined. A list of values is initially tried but found to be not very effective. Therefore a fixed value is given for each position.
2. A list of scale factors to be searched is created using ```scale_list = np.array(range(1,10,1))*0.2+1```, and is looped to find all the bounding windows possible. This list is determined on a trial-and-error basis and is a balance between efficiency and effectiveness.

#### 2. Show some examples of test images to demonstrate how your pipeline is working.  What did you do to optimize the performance of your classifier?

In the end the YCrCb 3-channel HOG features is chosen and is proved to provide a nice result.  Here are some example images:

![First Demo of Pipeline][image11]
![Second Demo of Pipeline][image12]

Several optimization techniques are implemented in the codes, such as:
1. Averaging the heat map of the most recent 40 frames for better stability and robustness.
2. Thresholding and masking the heat map to eliminate false positives.
3. Overlaying the heat map in the output for better visualization and easy debugging.
---

### Video Implementation

#### 1. Provide a link to your final video output.  Your pipeline should perform reasonably well on the entire project video (somewhat wobbly or unstable bounding boxes are ok as long as you are identifying the vehicles most of the time with minimal false positives.)

Here's a [link to my video result](./output_images/test_output.mp4), which is shared on [Youtube](https://youtu.be/dMt90mkirzw) as well.


#### 2. Describe how (and identify where in your code) you implemented some kind of filter for false positives and some method for combining overlapping bounding boxes.

In the 8th Code Cell in the [Jupyter Notebook](https://github.com/cedricxie/CarND-Vehicle-Detection/blob/master/vehicle_detection_YX.ipynb), a threshold of value 3 is applied on the heat map to remove areas of low confidence of having a car. In addition, a mask is applied on the heat map to remove the heat spots in the left one quarter of the image, because in this project video, this is the region where no cars shall be detected.

Moreover, the heat map is averaged for the most recent 40 frames in order to merge together overlapping bounding boxes.

---

### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

1. The biggest problem I faced is how to find the good features set to extract that will yield the most optimal results. It is indeed to my surprise that, under current setting, including color channel will actually lead to worse results. I looked through the forums as well as some suggestions/posts from other students, and they indeed helped me a lot. I will have to dig further into that issue if I have more time. 

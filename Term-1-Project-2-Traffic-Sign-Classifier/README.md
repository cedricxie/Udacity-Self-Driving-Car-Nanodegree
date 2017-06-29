# **Build a Traffic Sign Recognition Project**

The goals / steps of this project are the following:
* Load the data set (see below for links to the project data set)
* Explore, summarize and visualize the data set
* Design, train and test a model architecture
* Use the model to make predictions on new images
* Analyze the softmax probabilities of the new images
* Summarize the results with a written report


[//]: # (Image References)

[image1]: accuracy_baseline.png "Baseline"
[image2]: accuracy_gray_scaled.png "Gray Scaled"
[image3]: accuracy_normalized.png "Normalized"
[image4]: accuracy_normalized_and_gray_scaled.png "Normalized and Gray Scaled"
[image5]: accuracy_normalized_and_gray_scaled_epoch_50.png "Normalized and Gray Scaled with 50 Epoch"
[image6]: accuracy_normalized_and_gray_scaled_and_drop_out.png "Normalized and Gray Scaled with Dropout"
[image7]: data_information.png "Data Information"
[image8]: sign_before_gray_scale.png "Sign Before Gray Scaling"
[image9]: sign_after_gray_scale.png "Sign After Gray Scaling"
[image10]: new_image_1.jpg "New Sign: Children Crossing"
[image11]: new_image_2.jpg "New Sign: Speed Limit (20km/h)"
[image12]: new_image_3.jpg "New Sign: No Entry"
[image13]: new_image_4.jpg "New Sign: Stop"
[image14]: new_image_5.jpg "New Sign: Turn Right Ahead"
[image15]: new_image_2_top_wrong_answer.jpg "Wrong Answer: Dangerous curve to the right"

## Contents

---
### Writeup / README

You're reading it! and here is a link to my [project code](https://github.com/cedricxie/CarND-Traffic-Sign-Classifier-Project/blob/master/Traffic_Sign_Classifier.ipynb)

### Data Set Summary & Exploration

#### 1. Provide a basic summary of the data set.

I used the Numpy library to calculate summary statistics of the traffic
signs data set:

* The size of training set is 34799
* The size of the validation set is 4410
* The size of test set is 12630
* The shape of a traffic sign image is (32,32,1)
* The number of unique classes/labels in the data set is 43

#### 2. Include an exploratory visualization of the dataset.

Here is an exploratory visualization of the data set. These are the bar charts showing the probability of each sign in the training, validation and testing data set respectively. 

![Data Information][image7]

### Design and Test a Model Architecture

####1. Describe how you preprocessed the image data.

First, I decided to convert the images to grayscale because I think color is the minor factor in the sign, while the information from the sign is majorly coming from the shape and pattern. Therefore reducing RGB to gray scale could reduce redundant information and increase accuracy.

Here is an example of a traffic sign image before and after grayscaling.

![Sign Before Gray Scaling][image8]
![Sign After Gray Scaling][image9]

Second, I normalized the image data because it could help the data to become well conditioned before the training and therefore improves its prediction accuracy.

#### 2. Describe what your final model architecture looks like.

My final model consisted of the following layers:

| Layer         		|     Description	        					| 
|:---------------------:|:---------------------------------------------:| 
| Input         		| 32x32x1 Gray Scaled image   	    			| 
| Convolution 5x5     	| 1x1 stride, VALID padding, outputs 28x28x6	|
| RELU					|												|
| Max pooling	      	| 2x2 stride,  outputs 14x14x6   				|
| Convolution 5x5     	| 1x1 stride, VALID padding, outputs 10x10x16	|
| RELU					|												|
| Max pooling	      	| 2x2 stride,  outputs 5x5x6   			    	|
| Flatten   	      	| outputs 400				   			    	|
| Fully connected		| outputs 120  									|
| Dropout   	      	| Keep rate: 0.75			   			    	|
| Fully connected		| outputs 84  									|
| Dropout   	      	| Keep rate: 0.75			   			    	|
| Fully connected		| outputs 43  									|
 


#### 3. Describe how you trained your model.

To train the model, I used parameters as follows.
* Constant learning rate = 0.001
* Number of epochs = 20/50
* Batch size = 128
* Loss function = Softmax + Cross Entropy
* Optimizer = AdamOptimizer (Very cool comparison between optimizers at [this link](https://stackoverflow.com/questions/36162180/gradient-descent-vs-adagrad-vs-momentum-in-tensorflow))

#### 4. Describe the approach taken for finding a solution and getting the validation set accuracy to be at least 0.93.

My final model results were:
* training set accuracy of 0.964
* validation set accuracy of 0.946
* test set accuracy of 0.911

I chose LeNat-5 DNN model as the architecture.
* Reason： It works out pretty well on the task of image recognition for MNIST data set, therefore it should be applicable to traffic sign recognition as well.
* Results: The accuracy it achived on the training, validation and test sets are all > 0.91 and are its performance is pretty consistence. 
* Improvement: I tried tweaking several conditions in the model, as well as modifying the architecture by adding two dropouts. The performance for each case are shown here for comparison. The motivation for adding dropouts is because the accuracy reaches the plateau after about 10 epochs and the model seems to encounter overfitting (too good accuracy on the training set while insufficient accuracy on the validation set). Therefore dropouts are added in the LeNat-5 architecture and it indeed improves the accuracy on the validation set.

![Baseline (Original LeNat-5 Architecture, RGB without Normalization)][image1]
![Gray Scaled][image2]
![Normalized][image3]
![Gray Scaled and Normalized][image4]
![Gray Scaled and Normalized with 50 EPOCH][image5]
![Gray Scaled and Normalized with Dropout][image6]

| Condition         						| Accuracy on Training Set	  	| Accuracy on Validation Set     | 
|:-----------------------------------------:|:-----------------------------:| :-----------------------------:| 
| Baseline         							| 0.990   	    			    | 0.889							 |
| Gray Scaled     							| 0.992  	    			    | 0.904 						 |
| Normalized								| 0.990   	    			    | 0.907							 |
| Gray Scaled and Normalized	      		| 0.991   	    			    | 0.912 						 |
| Gray Scaled and Normalized with 50 EPOCH  | 0.997   	    			    | 0.906							 |
| Gray Scaled and Normalized with Dropout	| **0.961**      			    | **0.942** 					 |


### Test a Model on New Images

#### 1. Choose five German traffic signs found on the web and provide them in the report. For each image, discuss what quality or qualities might be difficult to classify.

Here are five German traffic signs that I found on the [web](http://electronicimaging.spiedigitallibrary.org/data/journals/electim/927109/jei_22_4_041105_f010.png):

![New Sign: Children Crossing][image10] ![New Sign: Speed Limit (20km/h)][image11] ![New Sign: No Entry][image12] 
![New Sign: Stop][image13] ![New Sign: Turn Right Ahead][image14]

The first one could be difficult to classify due to the complex shape in the sign.

#### 2. Discuss the model's predictions on these new traffic signs and compare the results to predicting on the test set.

Here are the results of the prediction:

| Image			        |     Prediction	        					| 
|:---------------------:|:---------------------------------------------:| 
| Children crossing     | Beware of ice/snow   							| 
| Speed limit (20km/h)  | Dangerous curve to the right   				|
| No entry				| No entry										|
| Stop	      			| Stop      					 				|
| Turn right ahead		| Right-of-way at the next intersection    		|


The model was able to correctly guess 2 of the 5 traffic signs, which gives an accuracy of 40%. This result is worse than the accuracy on the test set of 91% accuracy.

#### 3. Describe how certain the model is when predicting on each of the five new images by looking at the softmax probabilities for each prediction. Provide the top 5 softmax probabilities for each image along with the sign type of each probability. 

For the first image of "Child crossing", the model is having a hard time predicting what it is. The top five soft max probabilities were

| Probability         	|     Prediction	        					| 
|:---------------------:|:---------------------------------------------:| 
| .39         			| Beware of ice/snow							| 
| .20     				| Dangerous curve to the right					|
| .20					| General caution								|
| .10	      			| Turn left ahead        		 				|
| .05				    | Slippery Road      							|


For the second image of "Speed limit (20km/h)", it is wrongly predicted as "Dangerous curve to the right".

| Probability         	|     Prediction	        					| 
|:---------------------:|:---------------------------------------------:| 
| .97         			| Dangerous curve to the right  				| 
| .01     				| Speed limit (120km/h)							|
| .01					| Keep right									|
| .01	      			| Ahead only	        		 				|
| .00				    | Roundabout mandatory 							|

For the third image of "No Entry":

| Probability         	|     Prediction	        					| 
|:---------------------:|:---------------------------------------------:| 
| .99         			| No entry									    | 
| .00     				| Stop											|
| .00					| Priority road									|
| .00	      			| Turn right ahead	       		 				|
| .00				    | Speed limit (30km/h)							|

For the fourth image of "Stop":

| Probability         	|     Prediction	        					| 
|:---------------------:|:---------------------------------------------:| 
| .85         			| Stop										    | 
| .11     				| Keep right									|
| .01					| Road work										|
| .01	      			| Bumpy road	       		 					|
| .01				    | Bicycles crossing								|

For the fifth image of "Turn right ahead", it is predicted to be "Right-of-way at the next intersection" by mistake.

| Probability         	|     Prediction	        					| 
|:---------------------:|:---------------------------------------------:| 
| .46         			| Right-of-way at the next intersection		    | 
| .23     				| Turn right ahead								|
| .11					| Priority road									|
| .04	      			| Roundabout mandatory	       					|
| .03				    | Beware of ice/snow							|
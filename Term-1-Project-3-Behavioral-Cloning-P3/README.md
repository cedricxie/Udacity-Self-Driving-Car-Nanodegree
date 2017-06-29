# **Behavioral Cloning** 

---

**Behavioral Cloning Project**

The goals / steps of this project are the following:
* Use the simulator to collect data of good driving behavior
* Build, a convolution neural network in Keras that predicts steering angles from images
* Train and validate the model with a training and validation set
* Test that the model successfully drives around track one without leaving the road
* Summarize the results with a written report


[//]: # (Image References)

[image1]: ./figures/PilotNet_NVIDIA.JPG "PilotNet from NVIDIA"
[image2]: ./figures/model_loss.png "Model Loss"
[image3]: ./figures/center_image.jpg "Center Image"
[image4]: ./figures/left_image.jpg "Left Image"
[image5]: ./figures/right_image.jpg "Right Image"
[image6]: ./figures/center_image_flip.png "Flipped Image"
[image7]: ./figures/center_image_crop.png "Cropped Image"

## Rubric Points
---
### Files Submitted & Code Quality

#### 1. Submission includes all required files and can be used to run the simulator in autonomous mode

My project includes the following files in addition to the original files from Udacity:
* `model.py` A Pythong script to create and train the model
* `model.h5` A trained convolution neural network 
* `Behavior_Cloning_YX.ipynb` A Jupyter notebook with final model
* `Video.mp4` A video file showing a car driving autonomously for a full lap. [Please also find it on Youtube following this link (resubmitted version 2)](https://youtu.be/IKuauaKf24M)
* `**README.md**` A report to summarize the results

#### 2. Submission includes functional code
Using the Udacity provided simulator and my drive.py file, the car can be driven autonomously around the track by executing 
```sh
python drive.py model.h5
```

#### 3. Submission code is usable and readable

The model.py file contains the code for training and saving the convolution neural network. The file shows the pipeline I used for training and validating the model, and it contains comments to explain how the code works.

### Model Architecture and Training Strategy

#### 1. An appropriate model architecture has been employed

My model implementes the PilotNet from Nvidia and its structure is similar to what NVIDIA presented in their paper, [Explaining How a Deep Neural Network Trained with End-to-End Learning Steers a Car](https://arxiv.org/pdf/1704.07911.pdf), as shown in the following figure.

![PilotNet from NVIDIA][image1]

Namely, its structure is described as follows:
1. A Lambda layer to normalize the data (model.py, code line 66). 
2. Five convolution layers, each followed by a RELU activation function (model.py, code line 67 - 71). 
3. A Flatten layer to make the data into one row in preparation of the fully connected layers (model.py, code line 73).
4. Four fully connected layers, with three Dropout layers in between (model.py, code line 74 - 80).

#### 2. Attempts to reduce overfitting in the model

The model contains three dropout layers in order to reduce overfitting (model.py lines 75, 77 and 79). 

The model was trained and validated on different data sets to ensure that the model was not overfitting (code line 84-86). The model was tested by running it through the simulator and ensuring that the vehicle could stay on the track.

#### 3. Model parameter tuning

The model used an adam optimizer, so the learning rate was not tuned manually (model.py line 83).

#### 4. Appropriate training data

Training data was chosen to keep the vehicle driving on the road. I used a combination of closewise and anticlockwise driving. Each with two laps.

### Model Architecture and Training Strategy

#### 1. Solution Design Approach

The ultimate goal of this project was to design a deep learning model so that car is able to drive autonomouly within the lanes after training.

My first step was to implement a deep neural network model similar to the [*PilotNet* model built at NVIDIA](https://arxiv.org/pdf/1704.07911.pdf). This is a model capable of mimicking human drivers in various road conditions by only observing the road images created by human drivers. This is also the model recommended by Udacity and I belive it will apply well in our project.

In order to gauge how well the model was working, I split my image and steering angle data into a training and validation set. I found that my first model had a low mean squared error on the training set but a high mean squared error on the validation set. This implied that the model was overfitting. Therefore I added several layers of Dropout between fully connected layers and then the performance on training and validation sets are comparable. I do find that the low accuracy in the model (around 0.3 to 0.4) does not necessarily mean that the model is doing bad, as it is still able to drive the car autonomously for a full lap.

Then I add the generator feature in the model so I can utilize more training data without having to worry about the excessive memory requirement.

The final step was to run the simulator to see how well the car was driving around track one. In addition to the original training data provided by Udacity, I added two laps for both clockwise and anticlockwise driving, as well as images from flipping and from left and right cameras, which adds to a total of 15020 images.

At the end of the process, the vehicle is able to drive autonomously around the track without leaving the road.

#### 2. Final Model Architecture

The final model architecture (model.py lines 66-85) consisted of a deep neural network with multiple convolusion and fully connected layers, as described in details in section "Model Architecture and Training Strategy".

#### 3. Creation of the Training Set & Training Process

To capture good driving behavior, I first recorded two laps driving in clockwise direction on track one using center lane driving. Here is three example images from center, left and right cameras respectively:

![Center Image][image3]
![Left Image][image4]
![Right Image][image5]

Then I repeated another two laps but in the anticlockwise directions.

To augment the data sat, I also flipped images from center camera. For example, here is an image that has then been flipped:

![Flipped Center Image][image6]

After the collection process, I had 15020 number of data points. I then preprocessed this data by cropping and normalizing the images. For example, here is an image that has been cropped:

![Cropped Center Image][image7]

I finally randomly shuffled the data set and put 20% of the data into a validation set. 

I used this training data for training the model. The validation set helped determine if the model was over or under fitting. The ideal number of epochs was determined to be 10 as a compromise between accuracy and time efficiency. I used an adam optimizer so that manually training the learning rate wasn't necessary. The following figure shows the loss evolution for both training and validation sets during 10 epochs.

![Model Loss][image2]


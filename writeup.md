## Project: Perception Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


# Required Steps for a Passing Submission:
1. Extract features and train an SVM model on new objects (see `pick_list_*.yaml` in `/pr2_robot/config/` for the list of models you'll be trying to identify). 
2. Write a ROS node and subscribe to `/pr2/world/points` topic. This topic contains noisy point cloud data that you must work with.
3. Use filtering and RANSAC plane fitting to isolate the objects of interest from the rest of the scene.
4. Apply Euclidean clustering to create separate clusters for individual items.
5. Perform object recognition on these objects and assign them labels (markers in RViz).
6. Calculate the centroid (average in x, y and z) of the set of points belonging to that each object.
7. Create ROS messages containing the details of each object (name, pick_pose, etc.) and write these messages out to `.yaml` files, one for each of the 3 scenarios (`test1-3.world` in `/pr2_robot/worlds/`).  See the example `output.yaml` for details on what the output should look like.  
8. Submit a link to your GitHub repo for the project or the Python code for your perception pipeline and your output `.yaml` files (3 `.yaml` files, one for each test world).  You must have correctly identified 100% of objects from `pick_list_1.yaml` for `test1.world`, 80% of items from `pick_list_2.yaml` for `test2.world` and 75% of items from `pick_list_3.yaml` in `test3.world`.
9. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1067/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Exercise 1, 2 and 3 pipeline implemented
#### 1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.

Here's a picture of the table top after applying pass through and RANSAC filtering with plane fitting. I used two pass through fitlers, the first along the z axis to remove everything but the table top and objects and the second along the x axis to remove the front of the table.

![image1](images/image1.png)


#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.  

The image below shows clustering and segmentation applied to the three objects in test 1.
![image2](images/image2.png)

#### 2. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.

I extracted features from the objects using 100 poses for each object. This resulted in an accuracy score of about 0.95. Here's my normalized confusion matrix.
![image6](images/image6.png)

I implemented recognition and applied it to test worlds 1, 2, and 3.

![image3](images/image3.png)
![image5](images/image5.png)
![image4](images/image4.png)


### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.

Please see the included yaml files for output from the three test scenarios.

### Discussion

Spend some time at the end to discuss your code, what techniques you used, what worked and why, where the implementation might fail and how you might improve it if you were going to pursue this project further.  

#### 1. Feature extraction and training

I used HSV color space for feature extraction to minimize variance from different lighting conditions.

I experimented with different numbers of iterations for feature extraction and generally found that more iterations lead to more accuracy, though returns begin to diminish at high numbers of iterations. It seems as though 100 iterations was sufficient for this task, but a higher number of iterations would ensure greater accuracy and may be needed with less differentiable objects. Since feature extraction is a one time operation, I see no downside to training with a larger number of features.

I also experimented with different kernel types for the SVN classifier. Although the literature suggested that RBF may provide more accurate boundaries, I saw higher accuracy from the linear kernel and therefore used that in my training.

#### 2. Filtering, Clustering & Segmentation

As described above, I added a second pass through filter to remove the sides of the drop boxes and the front edge of the table. Prior to adding this, the sides of the drop boxes were being detected as objects.

I experimented with the cluster sizes for object segmentation. Initially my min size was too large to detect the glue bottle and I had to lower it (I also decreased the voxel filter so more points would be in each object). This worked fine, but an improvement would be to design a more general algorithm that could detect objects with even fewer points, if present.

#### 3. Object recognition

This worked surprisingly well with my training data, although I noticed that every once in a while an object would be momentarily mis-recognized. More tuning of the training parameters and/or multiple passes on the recognition step could help reduce mis-recognition errors.

#### 4. YAML output

I added test_scene_num as a ROS parameter (following a suggestion on the forum) and stored it in a variable, so I didn't need to change code when I changed test worlds.

I created a dictionary with dropbox parameters and used the 'group' value as the key to lookup arm and place_pose.

#### 5. Future improvements

When I have time I would like to try the other challenges, including implementing full pick and place. In addition to the improvements noted above, I'd like to experiment with different ML algorithms (perhaps neural nets) for object recognition. 
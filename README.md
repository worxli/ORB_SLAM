# 3D Vision

## Run the code
Follow the instructions below on how to setup ORB SLAM. Then for running the code use:
	roslaunch ExampleGroovyOrNewer.launch
and 
	rosbag play --pause Data/fisheye.bag
	
@Marius, chömr irgendwie de BAG ihne geh?

## Our changes
### Fisheye camera model

### 2D2D Matching
Tracking.cc and Initializer.cc

### Bundle Adjustment

### 3D2D Relocalization
PnPSolver.cc

# ORB_SLAM

ORB-SLAM is a versatile and accurate Monocular SLAM solution able to compute in real-time the camera trajectory and a sparse 3D reconstruction of the scene in a wide variety of environments, ranging from small hand-held sequences to a car driven around several city blocks. It is able to close large loops and perform global relocalisation in real-time and from wide baselines.

See our project webpage: http://webdiis.unizar.es/~raulmur/orbslam/

###Related Publications:

[1] Raúl Mur-Artal, J. M. M. Montiel and Juan D. Tardós. **ORB-SLAM: A Versatile and Accurate Monocular SLAM System**. *IEEE Transactions on Robotics,* vol. 31, no. 5, pp. 1147-1163, 2015. 

DOI: 10.1109/TRO.2015.2463671.

Link to pdf: http://webdiis.unizar.es/~raulmur/MurMontielTardosTRO15.pdf


#1. License

ORB-SLAM is released under a GPLv3 license. Please note that we provide along ORB-SLAM a modified version of g2o and DBoW2 which are both BSD. 

For a closed-source version of ORB-SLAM for commercial purposes, please contact the authors. 

If you use ORB-SLAM in an academic work, please cite:

    @article{murAcceptedTRO2015,
      title={{ORB-SLAM}: a Versatile and Accurate Monocular {SLAM} System},
      author={Mur-Artal, Ra\'ul, Montiel, J. M. M. and Tard\'os, Juan D.},
      journal={IEEE Transactions on Robotics},
      volume={31},
      number={5},
      pages={1147--1163},
      doi = {10.1109/TRO.2015.2463671},
      year={2015}
     }


#2. Prerequisites (dependencies)

##2.1 Boost

We use the Boost library to launch the different threads of our SLAM system.

	sudo apt-get install libboost-all-dev 

##2.2 ROS
We use ROS to receive images from the camera or from a recorded sequence (rosbag), and for visualization (rviz, image_view). 
**We have tested ORB-SLAM in Ubuntu 12.04 with ROS Fuerte, Groovy and Hydro; and in Ubuntu 14.04 with ROS Indigo**. 
If you do not have already installed ROS in your computer, we recommend you to install the Full-Desktop version of ROS Fuerte (http://wiki.ros.org/fuerte/Installation/Ubuntu).

**If you use ROS Indigo, remove the depency of opencv2 in the manifest.xml.**

##2.3 g2o (included)
We use g2o to perform several optimizations. We include a modified copy of the library including only the components we need 
and also some changes that are listed in `Thirdparty/g2o/Changes.txt`. 
In order to compile g2o you will need to have installed CHOLMOD, BLAS, LAPACK and Eigen3.

	sudo apt-get install libsuitesparse-dev
	sudo apt-get install libblas-dev
	sudo apt-get install liblapack-dev
	sudo apt-get install libeigen3-dev

##2.4 DBoW2 (included)
We make use of some components of the DBoW2 library (https://github.com/dorian3d/DBoW2) for place recognition and feature matching. We include a modified copy of the library including only the components we need and also some modifications that are listed in `Thirdparty/DBoW2/LICENSE.txt`. 
It only depends on OpenCV, but it should be included in the ROS distribution.


#3. Installation

1. Make sure you have installed ROS and all library dependencies (boost, eigen3, cholmod, blas, lapack).

2. Clone the repository:

		git clone https://github.com/raulmur/ORB_SLAM.git ORB_SLAM
		
3. Add the path where you cloned ORB-SLAM to the `ROS_PACKAGE_PATH` environment variable (better if you add the export line to your .bashrc file)

4. Build g2o. Go into `Thirdparty/g2o/` and execute:

		mkdir build
		cd build
		cmake .. -DCMAKE_BUILD_TYPE=Release
		make 

	*Tip: To achieve the best performance in your computer, set your favorite compilation flags in line 97 and 98 of* `Thirdparty/g2o/CMakeLists.txt` 
		  (by default -03 -march=native)

5. Build DBoW2. Go into Thirdparty/DBoW2/ and execute:

		mkdir build
		cd build
		cmake .. -DCMAKE_BUILD_TYPE=Release
		make  

	*Tip: Set your favorite compilation flags in line 4 and 5 of* `Thirdparty/DBoW2/CMakeLists.txt` (by default -03 -march=native)

6. Build ORB_SLAM. In the ORB_SLAM root execute:

	**If you use ROS Indigo, remove the depency of opencv2 in the manifest.xml.**

		mkdir build
		cd build
		cmake .. -DROS_BUILD_TYPE=Release
		make

	*Tip: Set your favorite compilation flags in line 12 and 13 of* `./CMakeLists.txt` (by default -03 -march=native)

#4. Usage

**See section 5 to run the Example Sequence**.

1. Launch ORB-SLAM from the terminal (`roscore` should have been already executed):

		rosrun ORB_SLAM ORB_SLAM PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE

  You have to provide the path to the ORB vocabulary and to the settings file. The paths must be absolute or relative   to the ORB_SLAM directory.  
  We already provide the vocabulary file we use in `ORB_SLAM/Data/ORBvoc.txt.tar.gz`. Uncompress the file, as it will be loaded much faster.

2. The last processed frame is published to the topic `/ORB_SLAM/Frame`. You can visualize it using `image_view`:

		rosrun image_view image_view image:=/ORB_SLAM/Frame _autosize:=true

3. The map is published to the topic `/ORB_SLAM/Map`, the current camera pose and global world coordinate origin are sent through `/tf` in frames `/ORB_SLAM/Camera` and `/ORB_SLAM/World` respectively.  Run `rviz` to visualize the map:
	
	*in ROS Fuerte*:

		rosrun rviz rviz -d Data/rviz.vcg

	*in ROS Groovy or a newer version*:

		rosrun rviz rviz -d Data/rviz.rviz

4. ORB_SLAM will receive the images from the topic `/camera/image_raw`. You can now play your rosbag or start your camera node. 
If you have a sequence with individual image files, you will need to generate a bag from them. We provide a tool to do that: https://github.com/raulmur/BagFromImages.


**Tip: Use a roslaunch to launch `ORB_SLAM`, `image_view` and `rviz` from just one instruction. We provide an example**:

*in ROS Fuerte*:

	roslaunch ExampleFuerte.launch

*in ROS Groovy or a newer version*:

	roslaunch ExampleGroovyOrNewer.launch


#5. Example Sequence
We provide the settings and the rosbag of an example sequence in our lab. In this sequence you will see a loop closure and two relocalisation from a big viewpoint change.

1. Download the rosbag file:  
	http://webdiis.unizar.es/~raulmur/orbslam/downloads/Example.bag.tar.gz. 

	Alternative link: https://drive.google.com/file/d/0B8Qa2__-sGYgRmozQ21oRHhUZWM/view?usp=sharing

	Uncompress the file.

2. Launch ORB_SLAM with the settings for the example sequence. You should have already uncompressed the vocabulary file (`/Data/ORBvoc.txt.tar.gz`)

  *in ROS Fuerte*:

	  roslaunch ExampleFuerte.launch

	*in ROS Groovy or newer versions*:

	  roslaunch ExampleGroovyHydro.launch

3. Once the ORB vocabulary has been loaded, play the rosbag (press space to start):

		rosbag play --pause Example.bag


#6. The Settings File

ORB_SLAM reads the camera calibration and setting parameters from a YAML file. We provide an example in `Data/Settings.yaml`, where you will find all parameters and their description. We use the camera calibration model of OpenCV.

Please make sure you write and call your own settings file for your camera (copy the example file and modify the calibration)

#7. Failure Modes

You should expect to achieve good results in sequences similar to those in which we show results in our paper [1], in terms of camera movement and texture in the environment. In general our Monocular SLAM solution is expected to have a bad time in the following situations:
- Pure rotations in exploration
- Low texture environments
- Many (or big) moving objects, especially if they move slowly.

The system is able to initialize from planar and non-planar scenes. In the case of planar scenes, depending on the camera movement relative to the plane, it is possible that the system refuses to initialize, see the paper [1] for details. 

#8. Need Help?

If you have any trouble installing or running ORB-SLAM, contact the authors.


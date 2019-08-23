# Long Term Global Localization leveraging Linear Geometry(based on ORB-SLAM2)

## Temporal Dataset can be downloaded here
 - []()

## Updating ...
**Reference**: 

- [Alkaid-Benetnash/ORB_SLAM2](https://github.com/Alkaid-Benetnash/ORB_SLAM2)

- [raulmur/ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2)

- [raulmur/ORB_SLAM2 - Issue #19](https://github.com/raulmur/ORB_SLAM2/issues/19)

- [raulmur/ORB_SLAM2 - Issues #246](https://github.com/raulmur/ORB_SLAM2/issues/246)

- [poine/ORB_SLAM2 - http://recherche.enac.fr/~drouin/slam/orbslam2/](http://recherche.enac.fr/~drouin/slam/orbslam2/poine_orbslam2_04_07_16.tgz), which includes Python Interface.

- [MathewDenny/ORB_SLAM2](https://github.com/MathewDenny/ORB_SLAM2)

**Original Authors:** [Raul Mur-Artal](http://webdiis.unizar.es/~raulmur/), [Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/), [J. M. M. Montiel](http://webdiis.unizar.es/~josemari/) and [Dorian Galvez-Lopez](http://doriangalvez.com/) ([DBoW2](https://github.com/dorian3d/DBoW2))

**14 Jul 2017**: Binary format ORB vocabulary and Map save/load are now supported(See section 10 and 11).

**13 Jan 2017**: OpenCV 3 and Eigen 3.3 are now supported.

**22 Dec 2016**: Added AR demo (see section 7).

ORB-SLAM2 is a real-time SLAM library for **Monocular**, **Stereo** and **RGB-D** cameras that computes the camera trajectory and a sparse 3D reconstruction (in the stereo and RGB-D case with true scale). It is able to detect loops and relocalize the camera in real time. We provide examples to run the SLAM system in the [KITTI dataset](http://www.cvlibs.net/datasets/kitti/eval_odometry.php) as stereo or monocular, in the [TUM dataset](http://vision.in.tum.de/data/datasets/rgbd-dataset) as RGB-D or monocular, and in the [EuRoC dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) as stereo or monocular. We also provide a ROS node to process live monocular, stereo or RGB-D streams. **The library can be compiled without ROS**. ORB-SLAM2 provides a GUI to change between a *SLAM Mode* and *Localization Mode*, see section 9 of this document.

<a href="https://www.youtube.com/embed/ufvPS5wJAx0" target="_blank"><img src="http://img.youtube.com/vi/ufvPS5wJAx0/0.jpg" 
alt="ORB-SLAM2" width="240" height="180" border="10" /></a>
<a href="https://www.youtube.com/embed/T-9PYCKhDLM" target="_blank"><img src="http://img.youtube.com/vi/T-9PYCKhDLM/0.jpg" 
alt="ORB-SLAM2" width="240" height="180" border="10" /></a>
<a href="https://www.youtube.com/embed/kPwy8yA4CKM" target="_blank"><img src="http://img.youtube.com/vi/kPwy8yA4CKM/0.jpg" 
alt="ORB-SLAM2" width="240" height="180" border="10" /></a>


### Related Publications:

- [Monocular] Raúl Mur-Artal, J. M. M. Montiel and Juan D. Tardós. **ORB-SLAM: A Versatile and Accurate Monocular SLAM System**. *IEEE Transactions on Robotics,* vol. 31, no. 5, pp. 1147-1163, 2015. (**2015 IEEE Transactions on Robotics Best Paper Award**). **[PDF](http://webdiis.unizar.es/~raulmur/MurMontielTardosTRO15.pdf)**.

- [Stereo and RGB-D] Raúl Mur-Artal and Juan D. Tardós. **ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras**. *ArXiv preprint arXiv:1610.06475* **[PDF](https://128.84.21.199/pdf/1610.06475.pdf)**.

- [DBoW2 Place Recognizer] Dorian Gálvez-López and Juan D. Tardós. **Bags of Binary Words for Fast Place Recognition in Image Sequences**. *IEEE Transactions on Robotics,* vol. 28, no. 5, pp.  1188-1197, 2012. **[PDF](http://doriangalvez.com/php/dl.php?dlp=GalvezTRO12.pdf)**

# 1. License

ORB-SLAM2 is released under a [GPLv3 license](https://github.com/raulmur/ORB_SLAM2/blob/master/License-gpl.txt). For a list of all code/library dependencies (and associated licenses), please see [Dependencies.md](https://github.com/raulmur/ORB_SLAM2/blob/master/Dependencies.md).

For a closed-source version of ORB-SLAM2 for commercial purposes, please contact the authors: orbslam (at) unizar (dot) es.

If you use ORB-SLAM2 (Monocular) in an academic work, please cite:

    @article{murTRO2015,
      title={{ORB-SLAM}: a Versatile and Accurate Monocular {SLAM} System},
      author={Mur-Artal, Ra\'ul, Montiel, J. M. M. and Tard\'os, Juan D.},
      journal={IEEE Transactions on Robotics},
      volume={31},
      number={5},
      pages={1147--1163},
      doi = {10.1109/TRO.2015.2463671},
      year={2015}
     }

if you use ORB-SLAM2 (Stereo or RGB-D) in an academic work, please cite:

    @article{murORB2,
      title={{ORB-SLAM2}: an Open-Source {SLAM} System for Monocular, Stereo and {RGB-D} Cameras},
      author={Mur-Artal, Ra\'ul and Tard\'os, Juan D.},
      journal={arXiv preprint arXiv:1610.06475},
      year={2016}
     }

# 2. Prerequisites
We have tested the library in **Ubuntu 12.04**, **14.04** and **16.04**, but it should be easy to compile in other platforms. A powerful computer (e.g. i7) will ensure real-time performance and provide more stable and accurate results.

## C++11 or C++0x Compiler
We use the new thread and chrono functionalities of C++11.

## Pangolin
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

## OpenCV
We use [OpenCV](http://opencv.org) to manipulate images and features. Dowload and install instructions can be found at: http://opencv.org. **Required at leat 2.4.3. Tested with OpenCV 2.4.11, OpenCV 2.4.13 and OpenCV 3.2**.

## Eigen3
Required by g2o (see below). Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0**.

## DBoW2 and g2o (Included in Thirdparty folder)
We use modified versions of the [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition and [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations. Both modified libraries (which are BSD) are included in the *Thirdparty* folder.

## ROS (optional)
We provide some examples to process the live input of a monocular, stereo or RGB-D camera using [ROS](ros.org). Building these examples is optional. In case you want to use ROS, a version Hydro or newer is needed.

## Boost(optional)

Map save/load feature needs boost library and more specifically the`libboost_serialization` library.

See section 11

# 3. Building ORB-SLAM2 library and examples

Clone the repository:
```
git clone https://github.com/JiankaiSun-SJTU-MVIG-training/ORB_SLAM2_Enhanced.git ORB_SLAM2
```

We provide a script `build.sh` to build the *Thirdparty* libraries and *ORB-SLAM2*. Please make sure you have installed all required dependencies (see section 2). Execute:
```
cd ORB_SLAM2
chmod +x build.sh
./build.sh
```

This will create **libORB_SLAM2.so**  at *lib* folder and the executables **mono_tum**, **mono_kitti**, **rgbd_tum**, **stereo_kitti**, **mono_euroc** and **stereo_euroc** in *Examples* folder.


# 4. Processing your own sequences
You will need to create a settings file with the calibration of your camera. See the settings file provided for the TUM and KITTI datasets for monocular, stereo and RGB-D cameras. We use the calibration model of OpenCV. See the examples to learn how to create a program that makes use of the ORB-SLAM2 library and how to pass images to the SLAM system. Stereo input must be synchronized and rectified. RGB-D input must be synchronized and depth registered.

# 5. SLAM and Localization Modes
You can change between the *SLAM* and *Localization mode* using the GUI of the map viewer.

### SLAM Mode
This is the default mode. The system runs in parallal three threads: Tracking, Local Mapping and Loop Closing. The system localizes the camera, builds new map and tries to close loops.

### Localization Mode
This mode can be used when you have a good map of your working area. In this mode the Local Mapping and Loop Closing are deactivated. The system localizes the camera in the map (which is no longer updated), using relocalization if needed. 

# 6. Binary Format ORB Vocabulary

You can load ORB vocabulary in either text or binary format. The format is determined by suffix(.txt for text format and .bin for binary format).

`build.sh` will generate a text-to-binary convertor `bin_vocabulary` in `Vocabulary/` . You can also find it as a target in `CMakeLists.txt`.

`bin_vocabulary` will convert `./ORBvoc.txt` to `./ORBvoc.bin` and you can use the new `ORBvoc.bin` as  `PATH_TO_VOCABULARY`  wherever needed.

PS: binary format is loaded faster and text format is more human-readable.

# 7. Map Save/Load

#### Enable:

You can enable this feature by defining a new variable `USE_MAP_SAVE_LOAD` when running cmake.

For example, you can change `cmake .. -DCMAKE_BUILD_TYPE=Release` to `cmake .. -DCMAKE_BUILD_TYPE=Release -DUSE_MAP_SAVE_LOAD=1` after `echo "Configuring and building ORB_SLAM2 ..."` in `build.sh`.

But `CMakeCache.txt` should be deleted if you want to undefine this variable.

#### Usage:

This feature is integrated with `class System`. The path of mapfile can be set by adding `Map.mapfile: map.bin` to ORB_SLAM2's settings file. See the last few line of `Example/Monocular/TUM1.xml`.

To save a map, you need construct `ORB_SLAM2::System` with the last parameter be `true`. Then the `System` will save map to mapfile specified in setting file when `ShutDown`.

With a readable mapfile, map will be loaded automatically and `System` will run in localization mode, but you can change it to SLAM mode later.

If you set a mapfile but it doesn't exist, `System` will create new map.

mono_tum has been updated as a simple example of this functionality. An extra command line parameter(0 or 1) should be given to indicate whether you want to save map or not.

#### Implementation related:

I use boost_serialization library to serialize `Map`, `MapPoint`, `KeyFrame`,`KeyFrameDatabase`, `cv::Mat`, `DBoW2::BowVector`, `DBoW2::FeatureVector`. In brief, only the `ORBVector` isn't serialized.

This feature is tested with boost 1.64 and it works fine mostly. There is still some occasional segmentfault to dig in.

# 持续更新修改中
2016.12.20
# awesome-SLAM-list
===========================================
##Contents
- [Tutorials-SLAM](#Tutorials-SLAM)
	- [SLAM Tutorial & Survey](#SLAM Tutorial & Survey) 
	- [Computer Vision Books](#Computer Vision Books)
	- [Video & Courses](#Video & Courses)
- [Papers-SLAM](#Papers-SLAM)
	- [Visual Odometry](#Visual Odometry)
	- [ORB-SLAM](#ORB-SLAM)
	- [Mono-SLAM](#Mono-SLAM)
	- [LSD-SLAM](#LSD-SLAM)
	- [RGBD-SLAM](#RGBD-SLAM)
	- [ElasticFusion](#ElasticFusion)
	- [Others-SLAM](#Others-SLAM)
- [OpenSource-SLAM](#OpenSource-SLAM)
	- [OpenSource-SLAM](#OpenSourceSLAM)
	- [SLAM-Migration](#SLAM-Migration)
	- [OpenSource-Minimization](#OpenSourceMinimization) 
	- [Nearest Neighbor Search](#NearestNeighborSearch)
	- [Useful Lib](#UsefulLib)
- [Feature Detection & Description](#Features)
	- [Feature Papers](#Feature Papers)
	- [Feature Libs](#Feature Libs)  
- [Datasets](#Datasets)
- [License](#License)
- [Contributing](#Contributing)
- [Others](#Others)

<a name="Tutorials-SLAM"></a>
# Tutorials-SLAM
<a name="SLAM Tutorial & Survey"></a>
## SLAM Tutorial & Survey

[OpenSLAM](https://openslam.org) The OpenSLAM Team: Cyrill Stachniss, Udo Frese, Giorgio Grisetti 

[ICRA 2016 Aerial Robotics - (Visual odometry)](http://mrsl.grasp.upenn.edu/loiannog/tutorial_ICRA2016/VO_Tutorial.pdf) D. Scaramuzza

[Simultaneous Localization And Mapping: Present, Future, and the Robust-Perception Age](http://arxiv.org/pdf/1606.05830.pdf). C. Cadena, L. Carlone, H. Carrillo, Y. Latif, D. Scaramuzza, J. Neira, I. D. Reid, J. J. Leonard. "The paper summarizes the outcome of the workshop “The Problem of Mobile Sensors: Setting future goals and indicators of progress for SLAM” held during the Robotics: Science and System (RSS) conference (Rome, July 2015)."


[Visual Odometry: Part I - The First 30 Years and Fundamentals](http://rpg.ifi.uzh.ch/docs/VO_Part_I_Scaramuzza.pdf), D. Scaramuzza and F. Fraundorfer, IEEE Robotics and Automation Magazine, Volume 18, issue 4, 2011

[Visual Odometry: Part II - Matching, robustness, optimization, and applications](http://rpg.ifi.uzh.ch/docs/VO_Part_II_Scaramuzza.pdf), F. Fraundorfer and D. Scaramuzza, IEEE Robotics and Automation Magazine, Volume 19, issue 2, 2012

[MRPT SLAM](http://www.mrpt.org/List_of_SLAM_algorithms)

<a name="Computer Vision Books"></a>
## Computer Vision Books

[Multiple View Geometry in Computer Vision Second Edition](http://www.robots.ox.ac.uk/~vgg/hzbook/). Richard Hartley & Andrew Zisserman. 2004.

[Computer Vision: Algorithms and Applications](http://szeliski.org/Book/). R. Szeliski. 2010.

[Development of Scientific Applications with the Mobile Robot Programming Toolkit (MRPT)](http://www.mrpt.org/tutorials/the-mrpt-book/)

<a name="Video"></a>
## Video & Courses & Blogs
- Courses: Robotics Lecture Course (course code 333) | Author: Andrew Davison
[https://www.doc.ic.ac.uk/~ajd/Robotics/index.html](https://www.doc.ic.ac.uk/~ajd/Robotics/index.html)

- Courses: Robotics | Author: The University of Pennsylvania

  [https://www.coursera.org/specializations/robotics](https://www.coursera.org/specializations/robotics)

 - Robotics:Aerial Robotics
 - Robotics:Computational Motion Planning
 - Robotics:Mobility
 - Robotics:Perception
 - Robotics:Estimation and Learning
 - Robotics:Capstone

- Courses: 视觉SLAM十四讲 | Author: 高翔

   [https://github.com/gaoxiang12/slambook](https://github.com/gaoxiang12)

- Blogs: 视觉SLAM  | Author: 高翔 | Email:gaoxiang12@mails.tsinghua.edu.cn

  [http://www.cnblogs.com/gaoxiang12/tag/视觉SLAM/](http://www.cnblogs.com/gaoxiang12/tag/视觉SLAM/)
 
<a name="Papers-SLAM"></a>
#Papers-SLAM

<a name="Visual Odometry"></a>
## Visual Odometry (image based only)

- [Real-time simultaneous localisation and mapping with a single camera](https://www.doc.ic.ac.uk/~ajd/Publications/davison_iccv2003.pdf). A. J. Davison. ICCV 2003.

- [Visual odometry](http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.137.4025&rep=rep1&type=pdf). D. Nister, O. Naroditsky, and J. Bergen. CVPR 2004.

- [Real time localization and 3d reconstruction](http://maxime.lhuillier.free.fr/pCvpr06.pdf). E. Mouragnon, M. Lhuillier, M. Dhome, F. Dekeyser, and P. Sayd. CVPR 2006.

- [Parallel Tracking and Mapping for Small AR Workspaces](http://www.robots.ox.ac.uk/~gk/publications/KleinMurray2007ISMAR.pdf). G. Klein, D. Murray. ISMAR 2007.

- [Real-Time 6-DOF Monocular Visual SLAM in a Large-scale Environments](http://cvlab.hanyang.ac.kr/~jwlim/files/icra2014vslam.pdf). H. Lim, J. Lim, H. Jin Kim. ICRA 2014.

<a name="ORB-SLAM"></a>
## ORB-SLAM
- ORB-SLAM: a Versatile and Accurate Monocular SLAM System
- ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras

<a name="Mono-SLAM"></a>
## Mono-SLAM
- MonoSLAM: Real-Time Single Camera SLAM
- Inverse Depth Parametrization for Monocular SLAM

<a name="LSD-SLAM"></a>
## LSD-SLAM
- LSD-SLAM: Large-Scale Direct Monocular SLAM
- Large-Scale Direct SLAM for Omnidirectional Cameras
- Large-Scale Direct SLAM with Stereo Cameras
- Reconstructing Street-Scenes in Real-Time From a Driving Car

<a name="RGBD-SLAM"></a>
## RGBD-SLAM
- 3-D Mapping With an RGB-D Camera
- An Evaluation of the RGB-D SLAM System
- A Benchmark for the Evaluation of RGB-D SLAM Systems
- Real-time dense appearance-based SLAM for RGB-D sensors
- RGB-D Mapping: Using Depth Cameras for Dense 3D Modeling of Indoor Environments
- RGB-D mapping: Using Kinect-style depth cameras for dense 3D modeling of indoor environments

<a name="ElasticFusion"></a>
## ElasticFusion
- ElasticFusion: Dense SLAM Without A Pose Graph
- ElasticFusion: Real-Time Dense SLAM and Light Source Estimation

<a name="Others-SLAM"></a>
## Others-SLAM
- DTAM: Dense Tracking and Mapping in Real-Time
- Dense Visual SLAM for RGB-D Cameras
- KinectFusion: Real-Time Dense Surface Mapping and Tracking ∗
- Parallel Tracking and Mapping for Small AR Workspaces
- SVO: Fast Semi-Direct Monocular Visual Odometry


<a name="OpenSource-SLAM"></a>
# OpenSource-SLAM
<a name="OpenSourceSLAM"></a>
## OpenSource-SLAM
| Project | Sensor | Link | Language | License | Video |
| ------------  | -------- | --- | --- | --- | --- |
| [MonoSLAM](https://github.com/hanmekim/SceneLib2) | 单目 | https://github.com/hanmekim/SceneLib2 |  | | | |
| [PTAM](https://www.robots.ox.ac.uk/~gk/PTAM) | 单目 | https://www.robots.ox.ac.uk/~gk/PTAM |  |  | |
| [ORB-SLAM](http://webdiis.unizar.es/~raulmur/orbslam) | 单目为主 | http://webdiis.unizar.es/~raulmur/orbslam | C++ | GPLv3 | |
| [LSD-SLAM](http://vision.in.tum.de/research/vslam/lsdslam) | 单目为主 | http://vision.in.tum.de/research/vslam/lsdslam | C++/ROS | GNU General Public License | |
| [SVO](https://github.com/uzh-rpg/rpg_svo) | 单目 | https://github.com/uzh-rpg/rpg_svo | C++/ROS | GNU General Public License | |
| [DTAM](https://github.com/anuranbaka/OpenDTAM) | RGB-D | https://github.com/anuranbaka/OpenDTAM | | | |
| [DVO](https://github.com/tum-vision/dvo_slam) | RGB-D | https://github.com/tum-vision/dvo_slam | | | |
| [DSO](https://github.com/JakobEngel/dso) | 单目 | https://github.com/JakobEngel/dso | | | |
| [RTAB-MAP](https://github.com/introlab/rtabmap) | 双目/RGB-D | https://github.com/introlab/rtabmap | | | |
| [RGBD-SLAM-V2](https://github.com/felixendres/rgbdslam_v2) | RGB-D | https://github.com/felixendres/rgbdslam_v2 | | | |
| [Elastic Fusion](https://github.com/mp3guy/ElasticFusion) | RGB-D | https://github.com/mp3guy/ElasticFusion | | | |
| [Hector SLAM](https://wiki.ros.org/hector_slam) | 激光 | https://wiki.ros.org/hector_slam | | | |
| [GMapping](https://wiki.ros.org/gmapping) | 激光 | https://wiki.ros.org/gmapping | | | |
| [OKVIS](https://github.com/ethz-asl/okvis) | 多目+IMU | https://github.com/ethz-asl/okvis | | | |
| [ROVIO](https://github.com/ethz-asl/rovio) | 多目+IMU | https://github.com/ethz-asl/rovio | | | |
| [COSLAM](http://drone.sjtu.edu.cn/dpzou/project/coslam.php) | | http://drone.sjtu.edu.cn/dpzou/project/coslam.php | C++| GNU General Public License | |
| [DTSLAM](https://github.com/plumonito/dtslam) | | https://github.com/plumonito/dtslam | C++ | Modified BSD | |
| [REBVO](https://github.com/JuanTarrio/rebvo) | | https://github.com/JuanTarrio/rebvo | C++ | GNU General Public License | 
| Swarm SLAM | | | | | |
| [RBPF-SLAM](http://www.mrpt.org/tutorials/slam-algorithms/rbpf-slam_algorithms/) | | | | | |
| HMT-SLAM | | | | | |
| [EKF-SLAM](http://www.mrpt.org/tutorials/supported_hardware_and_sensors/) | | | | | |
| [Graph-SLAM](http://www.mrpt.org/Graph-SLAM_maps) | | | | | |
| [ICP-SLAM](http://www.mrpt.org/tutorials/supported_hardware_and_sensors/) | | | | | |

<a name="SLAM-Migration"></a>
## SLAM-Migration

### IOS
| Project | Platform | Link | Language | License | Video |
| ------------  | -------- | --- | --- | --- | --- |
| [ygx2011/ORB_SLAM-IOS](https://github.com/ygx2011/ORB_SLAM-IOS) | iOS | https://github.com/ygx2011/ORB_SLAM-IOS | C++/Objective-C/Unity3D | | [Plane-SLAM-AR-ios](https://www.youtube.com/watch?v=rnODCkjPtq4) |
| [egoist-sx/ORB_SLAM_iOS](https://github.com/egoist-sx/ORB_SLAM_iOS) | iOS | https://github.com/egoist-sx/ORB_SLAM_iOS | C++/Objective-C | | |

### Android
| Project | Platform | Link | Language | License | Video |
| ------------  | -------- | --- | --- | --- | --- |
| [FangGet/ORB_SLAM2_Android](https://github.com/FangGet/ORB_SLAM2_Android) | Android | https://github.com/FangGet/ORB_SLAM2_Android | C++/java | | |

### ZED
| Project | Platform | Link | Language | License | Video |
| ------------  | -------- | --- | --- | --- | --- |
| [ygx2011/Stereo_SLAM_AR](https://github.com/ygx2011/Stereo_SLAM_AR) | OSX/ZED | https://github.com/ygx2011/Stereo_SLAM_AR | C++/Objective-C | | |
| [ygx2011/ZED_Stereo_ORBSLAM](https://github.com/ygx2011/ZED_Stereo_ORBSLAM) | OSX/ZED | https://github.com/ygx2011/ZED_Stereo_ORBSLAM | C++/Objective-C | | |

<a name="OpenSourceMinimization"></a>
## OpenSource Minimization

| Project |  Language | License |
| ---  | --- | --- |
|[G2O](https://github.com/RainerKuemmerle/g2o) | C++ |  BSD License + L/GPL3 restriction|
|[Ceres Solver](https://github.com/ceres-solver/ceres-solver) | C++ | BSD License|
|[GTSAM](https://collab.cc.gatech.edu/borg/gtsam) | C++ | BSD License|
|[NLopt](http://ab-initio.mit.edu/wiki/index.php/NLopt) | C++ | LGPL|

<a name="NearestNeighborSearch"></a>
## Nearest Neighbor Search

| Project |  Language | License|
| ---  | --- | --- |
|[ANN](http://www.cs.umd.edu/~mount/ANN/) | C++ | GNU General Public License|
|[Annoy](https://github.com/spotify/annoy) | C++ |  Apache License|
|[FLANN](http://www.cs.ubc.ca/research/flann/) | C++ | BSD License|
|[Nanoflann](https://github.com/jlblancoc/nanoflann) | C++ |  BSD License|

<a name="UsefulLib"></a>
## Useful Lib
### TODO:....
- Eigen
- ...

<a name="Features"></a>
# Feature Detection & Description
<a name="Feature Papers"></a>
## Feature Papers
Summarize: A survey of recent advances in visual feature detection
Yali Li a,b, Shengjin Wang a,b,n, Qi Tian c, Xiaoqing Ding a,b

<table>
	<tr>             
		<th width="200px"> Category </th>
		<th width="150px"> Classification </th>
		<th width="150px"> Methods </th>
		<th> Papers </th>
	</tr>
	<tr>
		<td rowspan="17">Blob Detection</td>    
		<td rowspan="14">PDE Based</td>
		<td> LoG </td>
		<td>  </td>
	</tr>
	<tr>
		<td> DoG </td>
		<td>  </td>
	</tr>
	<tr>
		<td> DoH </td>
		<td>  </td>
	</tr>
	<tr>
		<td> Hessian–Laplacian </td>
		<td>  </td>
	</tr>
	<tr>
		<td> SIFT </td>
		<td>  </td>
	</tr>
	<tr>
		<td> SURF </td>
		<td>  </td>
	</tr>
	<tr>
		<td> Cer-SURF </td>
		<td>  </td>
	</tr>
	<tr>
		<td> DART </td>
		<td>  </td>
	</tr>
	<tr>
		<td> Rank-SIFT </td>
		<td>  </td>
	</tr>
	<tr>
		<td> RLOG </td>
		<td>  </td>
	</tr>
	<tr>
		<td> MO-GP </td>
		<td>  </td>
	</tr>
	<tr>
		<td> KAZE </td>
		<td>  </td>
	</tr>
	<tr>
		<td> A-KAZE </td>
		<td>  </td>
	</tr>
	<tr>
		<td> WADE </td>
		<td>  </td>
	</tr>
	<tr>  
		<td rowspan="3">Template Based</td>
		<td> ORB </td>
		<td>  </td>
	</tr>
	<tr>
		<td> BRISK </td>
		<td>  </td>
	</tr>
	<tr>
		<td> FREAK </td>
		<td>  </td>
	</tr>
</table>                      

<a name="Feature Libs"></a>
## Feature Libs  
### TODO:....

| Project | Detection | Description |
| ---  | --- | --- |
|[AKAZE](https://github.com/pablofdezalc/akaze) |x|MSURF/MLDB|
|[DART](http://www.vision.cs.chubu.ac.jp/CV-R/pdf/MarimonCVPR2010.pdf) | x | x|
|[KAZE](https://github.com/pablofdezalc/kaze) |x|MSURF/MLDB|
|[LIOP/MIOP](https://github.com/foelin/IntensityOrderFeature) | |x|
|[LIFT (machine learning)](https://github.com/cvlab-epfl/LIFT) | x|x|
|MROGH | |x|
|SIFT |x|x|
|SURF |x|x|
|[SFOP](http://www.ipb.uni-bonn.de/data-software/sfop-keypoint-detector/) |x | |
|...  | | |

### "Real time" oriented methods

| Project | Detection | Description |
| ---  | --- | --- |
|BRIEF| |x|
|BRISK|x|x|
|FAST|x||
|FREAK| |x|
|FRIF|x|x|
|[HIPS](http://twd20g.blogspot.fr/2011/12/high-speed-feature-matching-with-simon.html) | | x|
|[LATCH](http://arxiv.org/pdf/1501.03719.pdf)| |x|
|MOPS | | x|
|[PhonySift](http://www.icg.tugraz.at/Members/gerhard/mvc/MVC_08_Tracking.pdf) | Multi-scale Fast | Reduced Sift grid|
|ORB|Multiscale Fast|Oriented BRIEF|

<a name="Datasets"></a>
# Datasets

- [Malaga Dataset](http://www.mrpt.org/MalagaUrbanDataset)
- [Tum: Computer Vision Lab: RGB-D](https://vision.in.tum.de/data/datasets/rgbd-dataset)
- [KITTI Dataset](http://www.cvlibs.net/datasets/kitti/)
- [University of Freiburg: Department of Computer Science](http://kaspar.informatik.uni-freiburg.de/~slamEvaluation/datasets.php)
- MRPT
- ICDL-NUIM

<a name="License"></a>
# License


<a name="Contributing"></a>
# Contributing

- 泡泡机器人SLAM: 微信公众号：paopaorobot_slam
- [高翔博士：https://github.com/gaoxiang12](https://github.com/gaoxiang12)
- [应高选：https://github.com/ygx2011](https://github.com/ygx2011)
- [awesome_3DReconstruction_list](https://github.com/openMVG/awesome_3DReconstruction_list)
- [Markdown——入门指南](http://www.jianshu.com/p/1e402922ee32/)
- [awesome-computer-vision](https://github.com/jbhuang0604/awesome-computer-vision)

<a name="Others"></a>
# Others
- [awesome-deep-learning](https://github.com/ChristosChristofidis/awesome-deep-learning)
- [awesome-maching-learning](https://github.com/josephmisiti/awesome-machine-learning)



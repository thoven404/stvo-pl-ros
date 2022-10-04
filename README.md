# stvo-pl-ros
Simple implementation of [PL-StVO](https://github.com/rubengooj/stvo-pl) using rviz in ros to visualize points, lines and tracks. (Test on ubuntu20.04)
<img src="https://github.com/zdf-star/stvo-pl-ros/blob/main/stvo-pl-ros.png" width = 100% height = 100% />

# PL-StVO #

This code contains an algorithm to compute stereo visual odometry by using both point and line segment features.

**Authors:** [Ruben Gomez-Ojeda](http://mapir.isa.uma.es/mapirwebsite/index.php/people/164-ruben-gomez) 
and [David Zuñiga-Noël](http://mapir.isa.uma.es/mapirwebsite/index.php/people/270)
and [Javier Gonzalez-Jimenez](http://mapir.isa.uma.es/mapirwebsite/index.php/people/95-javier-gonzalez-jimenez)

**Related publication:** [*Robust Stereo Visual Odometry through a Probabilistic Combination of Points and Line Segments*](http://mapir.isa.uma.es/mapirwebsite/index.php/people/164-ruben-gomez)

If you use PL-StVO in your research work, please cite:

    @InProceedings{Gomez2015,
      Title                    = {Robust Stereo Visual Odometry through a Probabilistic Combination of Points and Line Segments},
      Author                   = {Gomez-Ojeda, Ruben and Gonzalez-Jimenez, Javier},
      Booktitle                = {Robotics and Automation (ICRA), 2016 IEEE International Conference on},
      Year                     = {2016},
      Publisher                = {IEEE}
    }

The provided code is published under the General Public License Version 3 (GPL v3). More information can be found in the "GPU_LICENSE.txt" also included in the repository.

Please do not hesitate to contact the authors if you have any further questions.


## 1. Prerequisites and dependencies

### ros(*)
This project uses rviz in ros for visualization.
Install instructions can be found at: http://wiki.ros.org/

### OpenCV 4.x.x(*)
It can be easily found at http://opencv.org. 

### Eigen3
http://eigen.tuxfamily.org

### Boost
Installation on Ubuntu 16.04:
```bash
sudo apt-get install libboost-dev
```

### YAML
Installation on Ubuntu 16.04:
```bash
sudo apt install libyaml-cpp-dev
```

~~### MRPT~~

### Line Descriptor
We have modified the [*line_descriptor*](https://github.com/opencv/opencv_contrib/tree/master/modules/line_descriptor) module from the [OpenCV/contrib](https://github.com/opencv/opencv_contrib) library (both BSD) which is included in the *3rdparty* folder.



## 2. Configuration and generation
A CMakeLists.txt file is included to detect external dependencies and generate the project.

The project builds "imagesStVO", a customizable application where the user must introduce the inputs to the SVO algorithm, and then process the provided output. 



## 3. Usage

### Datasets configuration
~~We employ an environment variable, *${DATASETS_DIR}*, pointing the directory that contains our datasets. ~~Each sequence from each dataset must contain in its root folder a file named *dataset_params.yaml*, that indicates at least the camera model and the subfolders with the left and right images. We provide dataset parameters files for several datasets and cameras with the format *xxxx_params.yaml*.

### Configuration files
For running VO we can load the default parameters file or employ the *config_xxxx.yaml* files provided for every dataset.

### VO Application
Usage: ./imagesStVO <dataset_name> <Config_file>

A full command would be:
```cpp
./imagesStVO ~/kitti/00  ./src/stvo-pl/config/config/config_euroc.yaml

```


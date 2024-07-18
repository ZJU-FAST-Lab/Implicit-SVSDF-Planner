# Implicit-SVSDF-Planner
Code Release for SIGGRAPH 2024 & TOG

**Related Paper**: 

- [Implicit Swept Volume SDF: Enabling Continuous Collision-Free Trajectory Generation for Arbitrary Shapes](https://arxiv.org/pdf/2405.00362v1)
- Authors: Jingping Wang*, Tingrui Zhang*, Qixuan Zhang, Chuxiao Zeng, Jingyi Yu, Chao Xu, Lan Xu†, Fei Gao†.

  
* [Video on Bilibili](https://www.bilibili.com/video/BV1SJ4m1E7YF/?spm_id_from=333.999.0.0&vd_source=86f7d513b2e1d5a2a395a307e7802996)
* [Video on Youtube](https://youtu.be/XbcX-jPE89U)


<a href="https://www.bilibili.com/video/BV1SJ4m1E7YF/?spm_id_from=333.999.0.0&vd_source=86f7d513b2e1d5a2a395a307e7802996" target="blank">
  <p align="center">
    <img src="figs/Teaser.png" width="1000"/>
  </p>
</a>

## Abstract
In the field of trajectory generation for objects, ensuring continuous collision-free motion remains a huge challenge, especially for non-convex geometries and complex environments. Previous methods either oversimplify object shapes, which results in a sacrifice of feasible space or rely on discrete sampling, which suffers from the 'tunnel effect'. To address these limitations, we propose a novel hierarchical trajectory generation pipeline, which utilizes the **S**wept **V**olume **S**igned **D**istance **F**ield (SVSDF) to guide trajectory optimization for **C**ontinuous **C**ollision **A**voidance (CCA). Our interdisciplinary approach, blending techniques from graphics and robotics, exhibits outstanding effectiveness in solving this problem. We formulate the computation of the SVSDF as a Generalized Semi-Infinite Programming model, and we solve for the numerical solutions at query points implicitly, thereby eliminating the need for explicit reconstruction of the surface. Our algorithm has been validated in a variety of complex scenarios and applies to robots of various dynamics, including both rigid and deformable shapes. It demonstrates exceptional universality and superior CCA performance compared to typical algorithms.



  <div align="center">
  <img src="figs/a.gif" width="48%" />
  <img src="figs/b.gif" width="48%" />
  </div>
  <br>
  <div align="center">
  <img src="figs/c.gif" width="48%" />
  <img src="figs/d.gif" width="48%" />
  </div>
 

# Prerequisites
We use ros-neotic, which is a meta-package that includes several commonly used packages for robotics development. To install ros-neotic, follow the instructions in the [ros-neotic](http://wiki.ros.org/noetic/Installation/Ubuntu).
Make sure that the necessary Python packages catkin_pkg and empy(empy==3.3.4) are installed in your Python environment.For example, if the python environment in build.sh is set to "-DPYTHON_EXECUTABLE=/usr/bin/python3", you should have installed catkin_pkg under /usr/bin/python3:
```sh
sudo apt-get install gcc g++ make gfortran cmake libomp-dev
/usr/bin/python3 -m pip3 install pygame==2.0.0.dev12 
/usr/bin/python3 -m pip install catkin_pkg
/usr/bin/python3 -m uninstall em
/usr/bin/python3 -m install empy==3.3.4
```

## Qucik Start

```sh
git clone https://github.com/ZJU-FAST-Lab/Implicit-SVSDF-Planner.git
cd Implicit-SVSDF-Planner
./build.sh
source devel/setup.bash #(If use bash)
source devel/setup.zsh  #(If use zsh) 
roslaunch plan_manager xxxxx.launch 
```
We provide several launch files for different shapes. If loadStartEnd in someshape.yaml (src/plan_manager/config/) is set to true, the start and end points will be loaded automatically, otherwise, the start and end points will be selected by clicking on the 3D nav goal in rviz or pressing the 'G' key in the keyboard. 
```sh
roslaunch plan_manager run_sdTunnel.launch
roslaunch plan_manager run_star.launch
roslaunch plan_manager run_sdUnevenCapsule.launch
roslaunch plan_manager run_sdRoundedX.launch
roslaunch plan_manager run_sdRoundedCross.launch
roslaunch plan_manager run_sdRhombus.launch
roslaunch plan_manager run_sdPie2.launch
roslaunch plan_manager run_sdPie.launch
roslaunch plan_manager run_sdHorseshoe.launch
roslaunch plan_manager run_sdHeart.launch
roslaunch plan_manager run_sdCutDisk.launch
roslaunch plan_manager run_sdArc.launch
```
The shape variant demos will be released later...
## Tips
This is an extension of our previous work：
- [Continuous Implicit SDF Based Any-shape Robot Trajectory Optimization(IROS 2023)](https://github.com/ZJU-FAST-Lab/Implicit-SDF-Planner). Similarly, 
1. We use OpenMp for parallel acceleration, so the "threads_num" in the yaml should be adjusted to improve performance. The recommended threads_num is about 1.5 times the number of logical cores on the machine.
2. If users customize the shape, the obj file must be provided. We recommend using meshlab to generate obj files. For better performance, users can also implement corresponding SDF function, otherwise Libigl is used by default to compute the SDF.
3. The kernel_size multiplied by the map resolution should not be too small, this value should be greater than the maximum length of the robot's shape. So the "kernel_size" in the yaml should be adjusted accordingly (not too small).

## Licence

The source code is released under [MIT](https://en.wikipedia.org/wiki/MIT_License) license.

## Maintaince

For any technical issue or bug, please contact Tingrui Zhang (tingruizhang@zju.edu.cn) or Jingping Wang (22232111@zju.edu.cn).
For commercial inquiries, please contact [Fei GAO](http://zju-fast.com/fei-gao/) (fgaoaa@zju.edu.cn).

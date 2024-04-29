# Implicit-SVSDF-Planner
SIGGRAPH 2024 & TOG
Code Release: 
- Implicit Swept Volume SDF: Enabling Continuous Collision-Free Trajectory Generation for Arbitrary Shapes

This is an extension of our previous work：
- [Continuous Implicit SDF Based Any-shape Robot Trajectory Optimization(IROS 2023)](https://github.com/ZJU-FAST-Lab/Implicit-SDF-Planner)

* [Video on Youtube](https://youtu.be/XbcX-jPE89U)

<a href="https://youtu.be/XbcX-jPE89U" target="blank">
  <p align="center">
    <img src="figs/teaser.png" width="1000"/>
  </p>
</a>

## Abstract
In the field of trajectory generation for objects, ensuring continuous collision-free motion remains a huge challenge, especially for non-convex geometries and complex environments. Previous methods either oversimplify object shapes, which results in a sacrifice of feasible space or rely on discrete sampling, which suffers from the 'tunnel effect'. To address these limitations, we propose a novel hierarchical trajectory generation pipeline, which utilizes the **S**wept **V**olume **S**igned **D**istance **F**ield (SVSDF) to guide trajectory optimization for **C**ontinuous **C**ollision **A**voidance (CCA). Our interdisciplinary approach, blending techniques from graphics and robotics, exhibits outstanding effectiveness in solving this problem. We formulate the computation of the SVSDF as a Generalized Semi-Infinite Programming model, and we solve for the numerical solutions at query points implicitly, thereby eliminating the need for explicit reconstruction of the surface. Our algorithm has been validated in a variety of complex scenarios and applies to robots of various dynamics, including both rigid and deformable shapes. It demonstrates exceptional universality and superior CCA performance compared to typical algorithms.



  <div align="center">
  <img src="figs/1.gif" width="48%" />
  <img src="figs/2.gif" width="48%" />
  </div>
  <br>
  <div align="center">
  <img src="figs/3.gif" width="48%" />
  <img src="figs/4.gif" width="48%" />
  </div>
 

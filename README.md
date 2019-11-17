# V5++&Firefly提交规范

[V5++团队提交规范](http://git.npu5v5.cn/v5/Management/blob/master/团队规范/V5++提交规范.md)

# V5++&Firefly本项目进度管理方式

制定工作计划的流程是:
1. 开一个新工作安排的Issue(如果已经存在就不重新开)
2. Issue description中提到的每个人在下面发一下自己的安排,每一条头部添加复选框
3. 到截至日期的时候逐个检查打勾
4. 目前暂定每周制定进度一次

其他细则见[V5++工作管理方式](http://git.npu5v5.cn/v5/Management/blob/master/团队规范/V5++工作管理方式.md)

# V5++&Firefly本项目代码规范

> 推荐采用Clion进行开发，Clion默认有支持的.editorconfig的插件自动开启，且commit之前可勾选reformat进行自动格式化代码

本项目主要采用Google代码规范，请**确保**每次commit之前已将修改过的代码文件reformat！

.editorconfig文件已放入工程根目录中，请采用**支持.editorconfig的IDE**进行开发

**目前暂定，组内在它的框架上新添加的代码，请在按照它原来框架规范的namespace下再新建一个叫做firefly的namespace， 例如：**

```c++
namespace roborts_decision {
namespace firefly {

}
}
```

# RoboRTS

[![Build Status](https://travis-ci.org/RoboMaster/RoboRTS.svg?branch=master)](https://travis-ci.org/RoboMaster/RoboRTS)
[![Gitter](https://badges.gitter.im/RoboMaster/RoboRTS.svg)](https://gitter.im/RoboMaster/RoboRTS?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge)

## Introduction

<img src="images/robot.jpg" style="zoom:100%;display: inline-block; float:middle"/>

RoboRTS is an open source software stack for Real-Time Strategy research on mobile robots, developed by RoboMaster and motivated by [RoboMaster AI Challenge](#competition)

The framework of RoboRTS consists two parts: autonomous mobile robot layer and intelligent decision-making layer.

<img src="images/system.png" style="zoom:80%;display: inline-block; float:middle"/>

The autonomous mobile robot layer alone can let a robot, offically supported for RoboMaster AI Robot platform, to demonstrate a certain level of intelligence. On its on-board computer runs perception, motion planning, decision modules. It is fully supported in ROS with community driven as well as platform-customized codes and examples. On its MCU, an RT low-level robot controller is implemented to govern the robot driving system.  

**TODO:** Intelligent decision-making layer includes a multi-agent decision-making framework and a game simulator, it will be released soon in the future.

## Tutorial

For more information about RoboMaster AI Robot platform and RoboRTS framework, please refer to [RoboRTS Tutorial](https://robomaster.github.io/RoboRTS-Tutorial/#/)

## Competition

RoboMaster holds AI Challenge since 2017. In the challenge, multiple robots should fight with each other on a game field automatically under different rules.

For more information, please refer to

- [DJI RoboMaster 2019 ICRA AI Challenge](https://icra2019.org/competitions/dji-robomaster-ai-challenge)

- [DJI RoboMaster 2018 ICRA AI Challenge](https://icra2018.org/dji-robomaster-ai-challenge/)

## Copyright and License

RoboRTS is provided under the [GPL-v3](COPYING).

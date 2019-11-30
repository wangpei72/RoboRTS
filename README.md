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

# 2019款AI机器人平台使用说明

1. 机器人安装好DJI智能电池后，电池先短按再长按开启，然后开启电池后方的开机键
2. 根据是否使用裁判系统，长按车上的裁判系统模块的显示屏下的左上角按钮，进入设置界面。若不使用裁判系统对车进行激活，请进入`Debug Option/Function Module Mask`，启用`Offline Mode`，此时车上电机进入上电锁死状态，即说明此时车进入可控制移动的状态，可以通过遥控器或者代码控制
3. 遥控器右上角的拨键拨到最下面时，此时车是受代码控制状态；上面两个键位是遥控状态。所以车在代码控制失控的时候，可以瞬间切回来
4. 上位机按下左下角的开机键开机后，进入Ubuntu 16.04系统，登录密码目前是nwpu
5. 车子的上位机主要代码RoboRTS的放置路径为：`~/unstable_ws/src/roborts`。若要更改代码，可先在自己本地电脑上推到本仓库中，然后车的上位机连上校园网后可把代码从gitlab上把对应分支拉下来并切换到即可。拉下来后进行编译。这个workspace的环境变量的source步骤已加入到`.bashrc`里。
6. 车子此时如果要切换到代码控制，只需要将右上角遥控器拨键调到最下方即可。

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

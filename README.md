# V5++&Firefly提交规范

- 每个人工作的时候开一个个人分支，命名为每个人**姓名首字母缩写，全小写**，如薛轲翰的个人分支为xkh
- master分支**禁止**直接提交，个人分支需要提交的时候开pull request申请提交到**dev**分支上，并附带commit
- 每次提交的时候的commit中英文不限，但要表达清楚
- 至少每周提交一次。必须每日晚上下班前提交一次，且该次commit应加“weekly-backup"标注
- （后续待补充）

# V5++&Firefly本项目进度管理规范

制定工作计划的流程是:
1. 开一个新工作安排的Issue(如果已经存在就不重新开)
2. Issue description中提到的每个人在下面发一下自己的安排,每一条头部添加复选框
3. 到截至日期的时候逐个检查打勾


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

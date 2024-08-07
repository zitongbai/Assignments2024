# 任务一

本任务通过在Gazebo仿真环境中实现倒立摆的平衡，旨在让同学们熟悉ROS的基本操作，以及如何使用Gazebo仿真环境和ROS的一些基本工具，如rviz、ros control等。

本文档末尾含有一些参考资料，供大家参考。如遇到问题或者不熟悉的知识，请先查阅这些资料，再自行搜索。

## 任务概览

已为大家搭建好了一个简单的倒立摆仿真环境。该倒立摆包含一个沿着x轴移动的基座，一根杆以及一个连接在杆上的球。它一共有两个关节，一个是平移关节，另一个是旋转关节。其中旋转关节是欠驱动的，我们在控制时只能给平移关节施加力。

编译之后，运行如下的命令，可以进行可视化：

```bash
# 指令1
roslaunch inverted_pendulum_description ip_viz.launch
```

此时应该会弹出两个窗口，一个是rviz，用来显示倒立摆的模型，另一个是含有两条滑动条的窗口。你可以试着移动滑动条，感受一下这个倒立摆的运动自由度。

* 请注意：这里只是可视化，并没有进行仿真

关掉前面运行的程序，运行如下的命令，可以进行仿真：

```bash
# 指令2
roslaunch inverted_pendulum_gazebo ip_world.launch
```

若正常运行，应该会弹出一个Gazebo仿真界面，里面有一个倒立摆模型。
接着新开一个终端（前面的程序先不用停），运行如下的命令：

```bash
# 指令3
roslaunch ip_ctrl load_ros_controller.launch
```

该指令暂时不会产生显式的效果，后面会用到。

我们的任务目标是，**设计一个控制器，通过控制平移关节的力（即施加在基座上的水平力），使得倒立摆保持平衡（即竖直向上）**。

## 具体任务

### 1. 回答如下问题

请回答如下问题，并将问题的答案写在一个`answer.md`文件中并提交。这些问题对于ROS的开发非常重要，请同学们认真思考。可以使用Chatgpt等工具进行查询，但禁止直接复制粘贴，请广泛查阅资料后，整理成自己的答案再提交。

1. 阅读前述[任务概览](#任务概览)中第一个指令的launch文件以及其他相关文件，回答：
    - launch文件中`<param ...`那两行代码的作用是什么？（提示：可以先看看urdf，xacro等）
    - urdf文件和xacro文件的关系是什么？
    - `joint_state_publisher`和`robot_state_publisher`的作用是什么？（提示：思考它们分别接收什么信息，输出什么信息），以及为什么我们在这里需要它们？
2. 阅读前述[任务概览](#任务概览)中第二个指令的launch文件以及其他相关文件，回答：
    - launch文件为什么要设置`use_sim_time`为`true`？（提示：了解ROS中的时间机制）
    - launch文件中`<arg name="world_name" value=...`这行代码的作用是什么？
    - launch文件中`urdf_spawner`那个节点的作用是什么？（提示：可以对比有它和没有它的情况）
3. 阅读前述[任务概览](#任务概览)中第三个指令的launch文件以及其他相关文件，回答：
    - `rosparam`和前面第一个launch文件中出现的`param`有什么区别？
    - launch文件中`controller_spawner`的作用是什么？`args="joint_state_controller horizontal_joint_controller"`中的两个controller指的是什么、它们是在哪里被定义的？（提示：先学习ros control这个机制的基本原理）
    - launch文件中`remap`那一行代码的作用是什么？（提示：结合前面robot_state_publisher的知识，即思考一下输入户输出的信息）


### 2. 修改模型

修改某些文件中的某些参数，完成如下的设置：

* 设置水平方向上的移动为无阻尼的
* 设置杆的长度为0.8m
* 目前模型中各个link的惯量矩阵均不准确，请你根据各个连杆的形状和质量，重新计算并填入正确的惯量矩阵
* 将倒立摆模型中顶部的球的颜色改为红色（在Gazebo中能显示红色即可）
* 将旋转关节的初始角度改为 -0.03 (rad)


### 3. 运行demo.py

在`ip_ctrl`这个包中，有一个`demo.py`文件，它提供了一个简单的示例，展示如何读取倒立摆的状态，以及如何给倒立摆施加力。

这个文件中已经有了一些代码，你需要完成以下任务：

* 在`demo.py`中，找到`TODO`标记的地方，并按照要求完成代码
* 完成之后，运行`demo.py`（提示：不是直接调用python运行它）

提示：
* 在两个终端中分别运行了前述的指令2和指令3之后，你才能在一个新的终端中运行`demo.py`。
* 记得source一下你的工作空间。

### 4. 编写控制器

参考`demo.py`中的代码，再在`ip_ctrl/src`中创建一个文件，编写一个控制器，使得倒立摆保持平衡（即竖直向上）。

控制方法不限，可以使用PID，也可以使用LQR等。

## 任务提交

请将你修改后的assignment_1文件夹以及`answer.md`文件打包成一个压缩包，命名为`assignment_1_你的学号_你的姓名.zip`，并提交到群里的云盘连接中。

压缩包结构如下：

```
.
├── assignment_1
│   ├── inverted_pendulum_description
│   ├── inverted_pendulum_gazebo
│   ├── ip_ctrl
│   └── README.md
└── answer.md
```


## 参考资料：

- [倒立摆模型](https://en.wikipedia.org/wiki/Inverted_pendulum)
- [urdf](https://wiki.ros.org/urdf/XML/Transmission)
- [xacro](https://wiki.ros.org/xacro)
- [joint_state_publisher](http://wiki.ros.org/joint_state_publisher)
- [robot_state_publisher](http://wiki.ros.org/robot_state_publisher)
- Gazebo仿真环境
  - [URDF in Gazebo](https://classic.gazebosim.org/tutorials?tut=ros_urdf&cat=connect_ros)
  - [ROS control in Gazebo](https://classic.gazebosim.org/tutorials?tut=ros_control&cat=connect_ros)
- ros control
    - [一个很好的blog](https://fjp.at/posts/ros/ros-control/)
    - [ros control官方文档](http://wiki.ros.org/ros_control)
- [ros代码编写规范](https://github.com/leggedrobotics/ros_best_practices)(仅作参考，不作要求)

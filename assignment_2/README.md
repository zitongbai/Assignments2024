# 任务二

在任务二中，需要大家完成：

* 相机内参标定
* aruco码的识别
* 基于YOLOv5的目标检测


## 1. 相机内参标定

### 任务描述

相机内参标定是后续视觉任务的基础。请通过查找资料，了解相机内参标定的基本原理，并且使用`opencv`或者`matlab`等工具完成相机内参标定。

### 开发环境

安装ROS时自带了opencv，正常情况下不需要什么额外的配置环境。可以直接在python中调用opencv，通过如下的python代码检查是否有安装opencv，并查看其版本：

```python
import cv2
print(cv2.__version__)
```

由于较新版本中有关aruco码的函数可能有所变化，因此若opencv的版本大于4.6，则建议通过以下的命令降级opencv版本：
```bash
pip install opencv-python==4.6.0.66
```

注：可能需要将pip改成pip3，具体根据自己的环境而定。

### 任务要求

1. 请使用`opencv`或者`matlab`等工具，对你电脑上的摄像头进行内参标定。（提示：具体教程自己上网上找）
2. 保存标定结果，包括相机内参矩阵、畸变系数。


## 2. aruco码的识别

### 任务描述

aruco码是一种二维码，常用于辅助机器人的定位，经常可以在各种机器人开发相关的视频中看到。在这个任务中，我们需要识别aruco码，并获取其ID以及位置和姿态。

### 开发环境

同上，使用opencv，写python代码即可。


### 任务要求

1. 使用[ArUco markers generator](https://chev.me/arucogen/)生成一个aruco码，并打印出来。其中的参数可以自行选择，注意打印出来之后，测量一下aruco码的边长是否和你生成时设定的边长一致。
2. 调用你电脑上的摄像头，使用`opencv`对摄像头实时拍摄画面中的aruco码进行识别，获取其ID以及位置和姿态，并将其实时显示在一个窗口中。（提示：这里需要使用前面标定的内参，并且此步不需要依赖于ROS，单纯使用Python即可实现）
3. 了解ROS中tf的基本概念，在上一步的基础上，创建一个ROS package，并在其中实现一个节点，将aruco码相对与相机的位置和姿态实时更新到tf中。

要求实现该视频中所展示的效果，并且提交一个类似的视频（建议录屏软件：[OBS](https://obsproject.com/)）

【考核用】调用opencv识别aruco码，在ros中更新tf https://www.bilibili.com/video/BV186p6e7E9p

（欢迎 一键三连+关注 哈哈哈）

注意：此处禁止使用aruco_ros等已经封装好的库，需要自己用opencv实现（可以使用opencv中aruco相关的函数）。


## 3. 基于YOLOv5的目标检测

### 任务描述

目标检测是计算机视觉中的一个重要任务，YOLOv5是一个非常流行的目标检测算法。在这个任务中，我们需要使用YOLOv5对图像中的目标进行检测。

### 开发环境

首先要说明的是，本任务的开发不涉及到ROS。但是由于本任务涉及到GPU计算，因此不建议在wsl2或者虚拟机中进行开发。如果你安装了双系统，则可以在Ubuntu中进行开发；如果没安装，也可以在Windows中进行开发。

#### （1）安装Conda环境
为了方便环境管理，可以使用Anaconda或Miniconda来安装Python环境。推荐使用Miniforge，它是一个基于conda的Python环境管理工具。

这里先解释一下什么是Python环境：对于开发人员而言，不同的项目可能需要不同的Python环境，因为不同的项目可能需要不同的Python版本或者不同的Python库。为了方便管理这些环境，我们使用conda来管理Python环境。这些环境也可以称为conda环境，它们之间是相互独立的。

Ubuntu中自带了一个Python环境，我们安装ROS时，是将所有的ROS相关的Python包都安装到了这个Python环境中。但是，在很多情况下（尤其是当今很多AI相关的代码），我们需要使用到一些其他的Python包，这些包可能和ROS中的Python包有冲突，因此我们需要一个独立的Python环境来运行这些代码。

在使用之前，建议先上网上再查一下更详细的介绍和教程。

（如果你的电脑上已经安装了Anaconda，也可以跳过这一步，他们是一样的）安装Miniforge的方法为：从Miniforge的github仓库下载Miniforge的安装包：[Miniforge](https://github.com/conda-forge/miniforge#miniforge3)下载相应操作系统的安装包并安装。安装过程中遇到问yes or no类的问题都选择yes即可，安装路径也直接默认。

注：
* windows中需要打开miniforge prompt（后文也称其为终端）来使用Conda环境
* 安装完成后，需要重启终端来使环境变量生效。

安装完成后，建议将每次打开命令行时自动激活base环境的设置给取消掉，方法是：

```shell
conda config --set auto_activate_base false
```

打开命令行，输入以下命令：

```shell
conda create -n myenv python=3.10
```

这样就创建了一个名为myenv的conda环境，并且指定了Python的版本为3.10。这里的myenv可以自己取名。

激活这个环境：

```shell
conda activate myenv
```

**在开始安装和使用以下每一个库之前，确保激活了上面创建的conda环境！**

**在开始安装和使用以下每一个库之前，确保激活了上面创建的conda环境！**

**在开始安装和使用以下每一个库之前，确保激活了上面创建的conda环境！**

#### （2）安装PyTorch

根据你使用的操作系统，选择合适的安装方式：[Pytorch官网](https://pytorch.org/)。如果电脑上有GPU，推荐安装GPU版本的Pytorch。

例如，在一个有GPU的Linux系统上，可以使用以下命令安装Pytorch：

```shell
(myenv) $ pip3 install torch torchvision torchaudio
```

安装完成之后，检测一下是否安装成功：

```shell
(myenv) $ python -c "import torch; print(torch.__version__)"
```

若输出Pytorch的版本号（例如`2.3.1+cu121`），则安装成功。

#### （3）配置YoLoV5

请参考YOLOv5的Github仓库：[YOLOv5](https://github.com/ultralytics/yolov5)，按照其说明进行配置。（**不要**下载那个`pip install ultralytics`）

### 任务要求

1. 自己上到网上搜集足够多的足球照片，然后标注并形成符合YOLOv5格式的数据集，可以考虑使用[label-studio](https://github.com/HumanSignal/label-studio)

2. 利用你的数据集，训练YOLOv5模型。（提示：在官方提供的预训练好的权重上操作）

3. 使用你训练好的模型，对该[链接](https://bhpan.buaa.edu.cn/link/AA87E04AF8783B4E0FB55462F334DE5C2D)中的“测试视频.mp4”中的足球进行识别，实现类似链接中“样例”的效果。

注：你可能需要了解了解AI中的一些基本概念，如神经网络、训练集、验证集、测试集等。

## 任务提交

将你的代码和其他要求的文件压缩成一个压缩包，命名为`assignment_2_你的学号_你的姓名.zip`，并提交到群里的云盘连接中。

压缩包结构如下：

```
.
├── 内参矩阵.txt
├── 第二个任务中要求提交的视频
└── 第三个任务中使用YOLOv5识别后的视频.mp4
```

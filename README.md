# 综合图像处理系统

## 一、使用教程

（1）首先打开MATLAB命令界面，输入“config”选择“打开现有GUI”，选择目录下的“image1.fig”文件，点击右上角的run运行文件，用户界面如下：
![add image](https://github.com/LUXUS1/Image-Processing-System-MATLAB-GUI/blob/master/im/1594812959053.png)

（2）此界面通过通过三个面板来显示图像，首先点击“选择图像”，在原始图像框中显示；然后选择要处理的方式，点击前方按钮，在过渡图像相框和处理结果框中显示过渡图像和最终图像。

- 主要功能如下：

​     图像基本处理，包括图像去噪、直方图均衡化。

​     图像边缘检测，包括使用canny算子进行边缘检测，Hough变换进行直线检测。

​     图像分割处理，包括使用最大类间方差法、K-means均值法、Otsu方法分别对图像进行分割处理。

​     图像目标检测与识别，包括车牌识别和物体检测。

​     特征提取，包括SIFT特征提取，SURF特征提取与匹配。

​     主成分分析法进行人脸识别，包括两个文件夹：test、train，以及整个test文件夹下所有图识别准确率。

​     卷积神经网络集实现MNIST手写数据集训练。 

## 二、实现过程和结果 

### 1、图像基本处理

* 图像去噪

![1594892233196](C:\Users\LUXUS\AppData\Local\Temp\1594892233196.png)

* 直方图均衡化

![1594892359702](C:\Users\LUXUS\AppData\Local\Temp\1594892359702.png)

### 2、图像边缘检测

* 边缘检测

![1594892481107](C:\Users\LUXUS\AppData\Local\Temp\1594892481107.png)

* Hough变换

  ![1594892519533](C:\Users\LUXUS\AppData\Local\Temp\1594892519533.png)

### 3、图像分割

* 最大类间方差法

  ![1594892580629](C:\Users\LUXUS\AppData\Local\Temp\1594892580629.png)

* k-means均值法

  ![1594892626824](C:\Users\LUXUS\AppData\Local\Temp\1594892626824.png)

### 4、目标检测

* 车牌识别

  ![1594892845931](C:\Users\LUXUS\AppData\Local\Temp\1594892845931.png)

* 物体检测

  ![1594892866673](C:\Users\LUXUS\AppData\Local\Temp\1594892866673.png)

### 5、特征提取与匹配

* SIFT特征提取

  ![1594892928892](C:\Users\LUXUS\AppData\Local\Temp\1594892928892.png)

* SURF特征提取与匹配 

  首先点击“选择第一张图”选择一张图像，然后点击“选择第二张图”选择第二张图，效果如图表1所示；点击选择“SURF特征提取与匹配”，最后点击“特征提取”，效果如图表二所示。 

  ![1594892985181](C:\Users\LUXUS\AppData\Local\Temp\1594892985181.png)

  ![1594893037073](C:\Users\LUXUS\AppData\Local\Temp\1594893037073.png)

### 6、PCA人脸识别

首先点击“训练机器”选择train文件夹，其次点击“选择图像”在test文件夹中选择一幅图像，然后点击“识别图像”，会在train文件夹中选择一幅匹配度最高的图像显示；最后点击“识别率”选择test文件夹，会显示整个文件中的图像与train文件夹中的图像匹配的识别率 。

![1594893359556](C:\Users\LUXUS\AppData\Local\Temp\1594893359556.png)

### 7、CNN实现mnist手写数据集识别

点击“CNN”会依次显示：

![1594893463622](C:\Users\LUXUS\AppData\Local\Temp\1594893463622.png)

![1594893470291](C:\Users\LUXUS\AppData\Local\Temp\1594893470291.png)

![1594893476777](C:\Users\LUXUS\AppData\Local\Temp\1594893476777.png)

![1594893484550](C:\Users\LUXUS\AppData\Local\Temp\1594893484550.png)






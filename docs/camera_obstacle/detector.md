# Apollo检测器Detector



`Detector`功能模块通过神经网络从图像中检测障碍物，是后续`transformer`、`track`算法的数据基础。
该功能模块的接口类定义`modules/perception/camera/lib/interface/base_obstacle_detector.h`
![检测器功能接口](../../image\detector1.png)

`detector`部分实际的算法都隐含在网络结构中，实际比较麻烦的可能是初始化的相关过程,检测器的整体结构分支还用于车道线的检测，`track`的特征提取，是一个实现多任务学习的网络结构。

## 1. 初始化

### 1.1 相关配置文件

首先在用户接口配置文件：
`modules/perception/production/conf/perception/camera/fusion_camera_detection_component.pb.txt`
中设置`camera_obstacle_perception_conf`(相机障碍物检测)的配置路径和配置文件，默认如下：

```c++
camera_obstacle_perception_conf_dir : "/apollo/modules/perception/production/conf/perception/camera"
camera_obstacle_perception_conf_file : "obstacle.pt"
```

在上述`obstacle.pt`文件中配置`detector`功能模块的参数文件路径，默认如下：

```c++
detector_param {
  plugin_param{
    name : "YoloObstacleDetector" //功能实现类的名称
    root_dir : "/apollo/modules/perception/production/data/perception/camera/models/yolo_obstacle_detector"
    config_file : "config.pt"
  }
  camera_name : "front_6mm"
  #camera_name : "spherical_left_forward"
}
detector_param {
  plugin_param{
    name : "YoloObstacleDetector"
    root_dir : "/apollo/modules/perception/production/data/perception/camera/models/yolo_obstacle_detector"
    config_file : "config.pt"
  }
  camera_name : "front_12mm"
}
```

由于存在焦距为`6mm`,`12mm`两种相机，因此有两组参数，但其最终指向的是同一个检测器。上述路径中包含了`yolo_obstacle_detector`检测器参数配置(注意原路径中的`config.pt`实际默认链接的是`3d-r4-half-config.pt`),内部包含的主要参数有`模型名称，模型类型，模型权重，模型的网络结构，检测的目标类型，anchor的尺寸及网络其他的超参数等`，此处先不展开。

```c++
//3d-r4-half-config.pt
model_param {
  model_name: "./3d-r4-half"
  model_type: "RTNetInt8"	//网络inference框架
  weight_file: "deploy.model"
  proto_file: "deploy.pt"	//网络模型结构
  anchors_file: "anchors.txt"
  types_file: "types.txt"
  calibratetable_root: "./3d-r4-half"
  confidence_threshold: 0.4
  offset_ratio: 0.288889
  cropped_ratio: 0.711111
  resized_width: 1440
  aligned_pixel: 32
  min_2d_height: 10
  min_3d_height: 0.1
  ori_cycle: 2
  with_box3d: true
  light_swt_conf_threshold: 0
  light_vis_conf_threshold: 0
  with_lights: true
  with_ratios: false
  # num_areas: 4
  border_ratio: 0.01
}
net_param {
  det1_loc_blob: "loc_pred"
  det1_obj_blob: "obj_pred"
  det1_cls_blob: "cls_pred"
  det1_ori_conf_blob: "ori_conf_pred"
  det1_ori_blob: "ori_pred"
  det1_dim_blob: "dim_pred"
  input_blob: "data"
  feat_blob: "conv3_3" //特征提取的分支用于跟踪
}
nms_param {
  type: "NormalNMS"
  threshold: 0.5
  sigma: 0.4
  inter_cls_nms_thresh: 0.6
}
```

上述文件给出了网络相关超参数的定义以及网络分支功能接口，下面进行初始化程序的参数加载：

### 1.2 初始化程序

​		初始化接口由`FusionCameraDetectionComponent::Init()`进入，通过`InitAlgorithmPlugin`执行`ObstacleCameraPerception::Init()`,然后根据外部配置文件的参数进行`detector`功能模块的初始化。
​	(1)首先根据`obstacle.pt`中参数`camera_name`通过`SensorManager`功能类(该类只有一个实例)分别创建相机模型(此处为两个`6mm,12mm`)
​	(2)由`obstacle.pt`中的`plugin_param->name`确定`detector_ptr`指向该功能的实现类为`YoloObstacleDetector`
​	(3)执行功能类`YoloObstacleDetector`的初始化`YoloObstacleDetector::Init()`

​		网络结构的总体参数配置位于`modules/perception/camera/lib/obstacle/detector/yolo/proto/yolo.proto`中，并通过之前的外部配置`3d-r4-half-config.pt`进行修改得到`yolo_param_`。
​		初始化过程主要就是提取参数信息到`YoloObstacelDetector`类成员变量中，其中比较重要的是网络结构`inference`的构建以及输出`blob`的定义,同时最后还对用于`track`的**特征提取器**进行了初始化。

#### 1.2.1 InitNet()

```c++
// @brief: 通过yolo_param中配置的输入输出blob的名称，以及网络结构模型参数构建inference
// @param [in]: yolo_param,model_root(模型所在的根路径)
// @param [in/out]: inference_
bool InitNet(const yolo::YoloParam &yolo_param,
             const std::string &model_root);
```

内部主要通过配置的参数中对应的输入节点`input_blob`和输出节点(省略)，并根据`3d-r4-half-config.pt` 中`model_type`所配置的前向推断框架(默认为`RTNetInt8`)通过工厂模式创建`Inference`类的实例化对象 inference_ ，构造函数中加载权重文件以及网络参数。该工程中定义的网络前向推断框架还包括`CaffeNet,RTNet,RTNetInt8,PaddleNet`。

```c++
// @brief: 构造推断器inference_，指向具体的推断器类型(caffenet,rtnet,paddlenet)
// @param [in]: model_type(推断器类型),proto_file(模型结构),weight_file(权重文件)
//             ,model_root(模型所在的根路径)，input_names和output_names为对应网络输入和最终输出节点名称
// @output: 内部构造函数加载参数和模型文件(weight_map_,net_param_)，最终得到指向具体推断器类型的inference_
inference_.reset(inference::CreateInferenceByName(model_type, proto_file,
                                                    weight_file, output_names,
                                                    input_names, model_root));
```

通过得到的具体前向推断框架(`CaffeNet`,`RTNet`,...)，进行相应框架的初始化，不同的框架结构初始化过程可能不同，此处以默认的`RTNetInt8`为例:

- `inference_->Init(shape_map)`

  ```c++
  // @brief: 根据模型文件中的参数和结构构建RTNet前向推断器
  // @param [in]:input_blob及其形状的map (shape={1, height_, width_, 3})
  // @return: true or false
  // @output: 构造inference环境  _context
  bool Init(const std::map<std::string, std::vector<int>> &shapes) override;
  ```

-  `inference_->Infer()`

  ```c++
  // @brief: 根据创建的inference环境 _context进行前向推断过程
  void RTNet::Infer() {
  ...
      context_->enqueue(max_batch_size_, &buffers_[0], stream_, nullptr);
    	BASE_CUDA_CHECK(cudaStreamSynchronize(stream_));
  ...
  }
  ```

#### 1.2.2 InitYoloBlob()

```c++
// @brief: 将region_output.h中定义的yolo_blobs_ 指向inference中对应的blob
// @param [in]:net_param(网络的输入输出blob名称)
void InitYoloBlob(const yolo::NetworkParam &net_param);
```

#### 1.2.3 InitFeatureExtractor()

```c++
// @brief: 特征提取器初始化，创建特征提取器功能类`TrackingFeatureExtractor`
// @param [in]:模型根路径 model_root
// @return: true or false
// @note: 特征提取器参数配置文件由`feature_file`属性定义，默认为feature.pt
bool YoloObstacleDetector::InitFeatureExtractor(const std::string &root_dir) {
  ...
  feature_extractor_.reset(BaseFeatureExtractorRegisterer::GetInstanceByName(
      "TrackingFeatureExtractor"));
  if (!feature_extractor_->Init(feat_options)) {
    return false;
  }
  return true;
}
```

上面初始化了特征提取器`TrackingFeatureExtractor`,特征提取器相关外部配置参数如下，特征提取方法为`ROIPooling`,定义参数文件为：`modules/perception/camera/lib/feature_extractor/tfe/tracking_feature.proto`

```protobuf
// production/data/perception/camera/models/yolo_obstacle_detector/3d-r4-half/feature.pt
extractor {
  feat_type: ROIPooling
  roi_pooling_param {
      pooled_h: 3
      pooled_w: 3
      use_floor: true
  }
}
```

同时，`3d-r4-half-config.pt`中定义了特征提取采用` feat_blob: "conv3_3"` 卷积层。

初始化流程到这基本完成，所做的主要工作：
	(1)对于不同的camera分别建立detector
	(2)初步确定用什么模型，`3d-r4-half-config.pt`，其中包含了具体的模型名称，网络结构和网络参数的配置**文件路径**
	(3)根据配置文件的路径，完成所需的特定**推断器的初始化**
	(4)特征提取器的初始化

---------

## 2. 处理流程

​	初始化之后，`Transformer`模块算法的执行部分由`FusionCameraDetctionComponent::OnReceiveImage`监听节点调用，然后在其`InternalProc()`中的函数`Perception()`执行`detector->Detect(detector_options, frame)`调用`detector`模块的算法接口。

### 2.1 ResizeGPU

```c++
// @brief: 裁剪图像
// @param [in]:image(图像)，input_blob（网络输入接口）,src_width(原图像宽度)
// @note: 采用cuda编程于GPU中运行
inference::ResizeGPU(*image_, input_blob, frame->data_provider->src_width(),0);
```

处理方式类似于下图所示(数据仅供参考),程序具体执行`resize.cu`，利用了cuda编程。

![](../../image\detector3.png)

###  2.2 inference_->Infer()

网络的前向推断，不同的网络框架推断过程有一定差异，同时在初始化时已经介绍该函数，此处不再赘述。

### 2.3 get_objects_gpu()

```c++
// @brief: 将原始的输出节点信息进行相关转换最终得到frame->detected_objects中对应的属性值
// @param [in]:yolo_blob_(对应网络的输入输出节点)，types_(障碍物类型)，model_param(模型相关阈值参数)，
//			  light_vis_conf_threshold_,light_set_conf_threshold_:转向灯，刹车灯阈值
// @param [in/out]:frame->detected_objects:(检测得到的障碍物信息)
// @note: 功能通过cuda编程实现于region_output.cu
get_objects_gpu(yolo_blobs_, stream_, types_, nms_, yolo_param_.model_param(),
                  light_vis_conf_threshold_, light_swt_conf_threshold_,
                  overlapped_.get(), idx_sm_.get(), &(frame->detected_objects));
```

yolo3dr4网络的输出结点示意图：

![yolo3dr4输出](C:\Users\jia_z\Desktop\Apollo_note\image\detector2.png)

### 2.4 filter_bbox()

```c++
// @brief: 剔除不符合要求的检测框，检测的box高度小于阈值会被忽略
// @param [in]:min_dims_阈值（配置于3d-r4-half-config.pt中）
// @param [in/out]:frame->detected_objects:(检测得到的障碍物信息)
// @note: 该函数同时有cpu和gpu实现
filter_bbox(min_dims_, &(frame->detected_objects));
```

### 2.5 feature_extractor_->Extract()

```c++
// @brief: Roipooling层的输出经L2正则化后存于track_feature_blob中，提取检测到的障碍物的特则会给你
// @param [in]:feat_options 参数配置（是否进行normalized）
// @param [in/out]:frame
// @note: 
feature_extractor_->Extract(feat_options, frame);
```

过程类似于下图所示（仅供参考）：

<img src="../../image\detector5.png" style="zoom: 80%;" />

### 2.6 recover_bbox()

```c++
// @brief: 将之前截取的ROI区域内的图像(归一化到0~1)检测到的box恢复到原图像中，将坐标对应到图像中的像素坐标
// @param [in]:原图像的长宽(width,height),crop的补偿(offset_y_)
// @param [in/out]:frame->detected_objects
// @note: 该函数有gpu和cpu两种实现 
  recover_bbox(frame->data_provider->src_width(),
               frame->data_provider->src_height() - offset_y_, offset_y_,
               &frame->detected_objects);
```

### 2.7 post processing

后处理部分主要进行：

- recover alpha

- get area_id from visible_ratios

  <img src="C:\Users\jia_z\Desktop\Apollo_note\image\detector4.png" style="zoom:67%;" />

- clear cut off ratios

## 3. 补充材料

### 3.1 Kitti数据集格式：

<img src="..\..\image\kitti_label.png" style="zoom:67%;" />

​	标签描述：
> ```
> #Values    Name      Description
> ----------------------------------------------------------------------------
> 1    type         Describes the type of object: 'Car', 'Van', 'Truck',
>                   'Pedestrian', 'Person_sitting', 'Cyclist', 'Tram',
>                   'Misc' or 'DontCare'
> 1    truncated    Float from 0 (non-truncated) to 1 (truncated), where
>                   truncated refers to the object leaving image boundaries
> 1    occluded     Integer (0,1,2,3) indicating occlusion state:
>                   0 = fully visible, 1 = partly occluded
>                   2 = largely occluded, 3 = unknown
> 1    alpha        Observation angle of object, ranging [-pi..pi]
> 4    bbox         2D bounding box of object in the image (0-based index):
>                   contains left, top, right, bottom pixel coordinates
> 3    dimensions   3D object dimensions: height, width, length (in meters)
> 3    location     3D object location x,y,z in camera coordinates (in meters)
> 1    rotation_y   Rotation ry around Y-axis in camera coordinates [-pi..pi]
> 1    score        Only for results: Float, indicating confidence in
>                   detection, needed for p/r curves, higher is better.
> ```

### 3.2 CUDA编程

核函数使用方法：
```cpp
Function<<<griddim,blockdim,extern shared memory,GPU stream>>>(param...);
```

其中中间参数可以控制核函数运行所占用的资源：
`griddim`为调用的block数，`blockdim`为调用的thread数，后面两个参数分别表示动态定义共享内存大小和可使用的SM处理器数。

<img src="../../image\detector7.png" style="zoom:50%;" />

kernel函数的定义采用`__global__`修饰符修饰。

```cpp
resize_linear_kernel << < grid, block >> >
      (src.gpu_data(), dst->mutable_gpu_data(),
          origin_channel, origin_height, origin_width,
          stepwidth, height, width, fx, fy);
```



### 3.3 TensorRT

官方开发手册：https://docs.nvidia.com/deeplearning/sdk/tensorrt-developer-guide/index.html#c_topics

参考博客:https://www.cnblogs.com/vh-pg/p/11680658.html

模型从导入TensorRT到执行`inference`大致经过一下三个阶段:

- Network Definition
- Builder
- Engine

使用过程中首先通过`TensorRT`的全局方法`creatInferBuilder()`创建一个`IBuilder`类指针，然后由该指针调用`IBuilder`类创建`Network`和`Engine`类的指针。
`INetworkDefinition`类
`INetworkDefinition`类即为网络定义，可通过`IBuilder`类方法`creatNetwork()`返回其指针。
`ICudaEngine`
`ICudaEngine`类即为Engine，可通过`IBuilder`类方法`buildCudaEngine()`/`buildEngineWithConfig()`返回其指针。
Engine的运行需要一个运行时的环境，通过`createExecutionContext()`方法为对应的`ICudaEngine`生成一个                     `IExecutionContext`类型的运行环境context。

![](C:\Users\jia_z\Desktop\Apollo_note\image\detector6.png)

### 3.4 Blob





### 3.5 参考资料




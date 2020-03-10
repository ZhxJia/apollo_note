# Camera_obstacle_pipeline总体流程

​	本文综合相机障碍物处理的总体流程,接口为继承了`apollo::cyber::Componnet<>`的`FusionCameraDetectionComponent`组件。



[TOC]



## FusionCameraDetectionComponent的文件结构

组件类继承于`cyber::Component`,并通过`CYBER_REGISTER_COMPONENT(FusionCameraDetectionComponent)`注册到`Cyber`中，其主要的文件结构包括：

- 头文件:`Fusion_camera_detection_component.h`
- 实现文件:`Fusion_camera_detection_component.cc`
- 构建文件:`BUILD`
- DAG配置文件:`modules/perception/production/dag/dag_streaming_perception_camera.dag`
- Launch启动文件:`modules/perception/production/launch/perception_camera.launch` 

以上仅仅列举了与`Camera_obstacle_pipeline`相关的`dag`和`launch`文件，`lidar`和`radar`的组件同理。下面简要分析这几个文件的主要作用：

**构建文件BUILD**：
通过`bazel`将相关依赖文件编译成动态链接库文件`"libperception_component_camera.so"`

```protobuf
cc_binary(
    name = "libperception_component_camera.so",
    linkshared = True,
    linkstatic = False,
    deps = [":perception_component_inner_camera"],
)
```

**DAG配置文件**：
根据`BUILD`生成的动态链接库，通过`mianboard`动态加载组件:

```python
module_config {
  #共享库文件路径
  module_library : "/apollo/bazel-bin/modules/perception/onboard/component/libperception_component_camera.so"
  components {
	#组件名称，mainboard动态加载
    class_name : "FusionCameraDetectionComponent"
    config {
	  #模块名称
      name: "FusionCameraComponent"
	  #绝对路径，配置文件路径
      config_file_path: "/apollo/modules/perception/production/conf/perception/camera/fusion_camera_detection_component.pb.txt"
      #预定义宏的参数配置文件（例如内参配置文件路径）
      flag_file_path: "/apollo/modules/perception/production/conf/perception/perception_common.flag"
    }
  }
}
```

> 需要注意的是有些dag文件中还会有readers{..}这个选项，这表示为组件`Proc`函数中接受的消息，camera这里没有用到`proc`函数，因此没有这一项,下面给出一个示例位于`dag_streaming_perception.dag`中
>
> ```c++
>       readers {
>           channel: "/apollo/sensor/lidar128/compensator/PointCloud2"
>         }
> ```

**Launch启动文件：**
根据`dag`文件加载相应启动模块,并添加进程名。

```xml
<!--this file list the modules which will be loaded dynamicly and
    their process name to be running in -->

<cyber>
    <desc>cyber modules list config</desc>
    <version>1.0.0</version>

    <!-- sample module config, and the files should have relative path like
         ./bin/cyber_launch
         ./bin/mainboard
         ./conf/dag_streaming_0.conf -->

    <module>
        <name>perception_camera</name>
        <dag_conf>/apollo/modules/perception/production/dag/dag_streaming_perception_camera.dag</dag_conf>
        <!-- if not set, use default process -->
        <process_name></process_name>
        <version>1.0.0</version>
    </module>

    <module>
        <name>motion_service</name>
        <dag_conf>/apollo/modules/perception/production/dag/dag_motion_service.dag</dag_conf>
        <!-- if not set, use default process -->
        <process_name></process_name>
        <version>1.0.0</version>
    </module>
</cyber>
```



## FusionCameraDetectionComponent中消息的接收和发布

### **接收消息**

基于`Cyber RT`接收消息分为两种：

- 虚函数：Proc()中处理指定的消息类型，周期性触发（接收），但最多只能接收4种消息类型（由cyber::Component的模板参数最多只有4个决定），一般用于模块主要输入信息的接收。

- 直接创建消息接收器，一般用于接收非周期性消息或模块的次要输入消息，例如：

  `modules/perception/onboard/component/fusion_camera_detection_component.cc`

  ```c++
  auto camera_reader = node_->CreateReader(channel_name, camera_callback);
  ```

在`FusionCameraDetctionComponent`组件中主要存在两类消息需要接收：

**(1)MotionService**

创建`reader`,并给出其回调函数。

```c++
//InitMotionservice()
const std::string &channel_name_local = "/apollo/perception/motion_service";
std::function<void(const MotionServiceMsgType &)> motion_service_callback =
    std::bind(&FusionCameraDetectionComponent::OnMotionService, this,
              std::placeholders::_1);
auto motion_service_reader =
    node_->CreateReader(channel_name_local, motion_service_callback);
```

对应的回调函数：

```c++
// On receiving motion service input, convert it to motion_buff_
void FusionCameraDetectionComponent::OnMotionService(
    const MotionServiceMsgType &message) {
    ...
    motion_buffer_->push_back(vehicledata);
    }
```

消息格式定义位于:`modules/perception/proto/motion_service.proto` ,主要包含了车辆底盘运动的相关信息，此处不展开

**(2)CameraListeners**

创建`reader`,并给出回调函数,输入图像的通道名称有：
`input_camera_channel_names : "/apollo/sensor/camera/front_6mm/image,/apollo/sensor/camera/front_12mm/image"`

```c++
//InitCameraListeners
typedef std::shared_ptr<apollo::drivers::Image> ImageMsgType;
std::function<void(const ImageMsgType &)> camera_callback =
    std::bind(&FusionCameraDetectionComponent::OnReceiveImage, this,
              std::placeholders::_1, camera_name);//jac!!20/1/14:回调
auto camera_reader = node_->CreateReader(channel_name, camera_callback);
```

对应回调函数：

```c++
//此函数为camera obstacle detect主要算法的处理函数
void FusionCameraDetectionComponent::OnReceiveImage(
    const std::shared_ptr<apollo::drivers::Image> &message,
    const std::string &camera_name) {
    ...
        
    }
```

在`modules/drivers/proto/sensor_image.proto`定义了Image的消息类型格式。

```c++
message Image {
  optional apollo.common.Header header = 1;
  optional string frame_id = 2;
  optional double measurement_time = 3;

  optional uint32 height = 4;  // image height, that is, number of rows
  optional uint32 width = 5;   // image width, that is, number of columns

  optional string encoding = 6;
  optional uint32 step = 7;  // Full row length in bytes
  optional bytes data = 8;   // actual matrix data, size is (step * rows)
}
```



### **发布消息**

基于`Cyber RT` 发布消息

![](C:\Users\jia_z\Desktop\Apollo_note\image\writer.png)

该组件中消息发布的节点(writer)共有四种，相关的创建过程和消息格式的定义文件如上图所示，通道名称可以在
`fusion_camera_detection_component.pb.txt`中设置，消息格式的定义基本都在`modules/perception/proto/..`文件目录下找到。

重点说明一下`prefused_message`即`sensorframe_writer`

```c++
 std::shared_ptr<SensorFrameMessage> prefused_message(new (std::nothrow)
                                                           SensorFrameMessage);
```

该`message`应该是用于后期的多传感器融合算法，其消息格式定义类：

```c++
class SensorFrameMessage {
 public:
  SensorFrameMessage() { type_name_ = "SensorFrameMessage"; }
  ~SensorFrameMessage() = default;
  std::string GetTypeName() { return type_name_; }
  SensorFrameMessage* New() const { return new SensorFrameMessage; }

 public:
  apollo::common::ErrorCode error_code_ = apollo::common::ErrorCode::OK;

  std::string sensor_id_;
  double timestamp_ = 0.0;
  uint32_t seq_num_ = 0;
  std::string type_name_;
  base::HdmapStructConstPtr hdmap_;

  base::FramePtr frame_; //包含了所有传感器的帧格式，以及object的相关属性.
  //当前的处理阶段
  ProcessStage process_stage_ = ProcessStage::UNKNOWN_STAGE; 
};
```

该消息类型可视为封装了所有传感器的检测信息，在相机检测阶段其主要参数的赋值位于`InternalProc`这个函数中。

```c++
int FusionCameraDetectionComponent::InternalProc(
  const std::shared_ptr<apollo::drivers::Image const> &in_message,
  const std::string &camera_name, apollo::common::ErrorCode *error_code,
  SensorFrameMessage *prefused_message,
  apollo::perception::PerceptionObstacles *out_message) {
  ...  
  prefused_message->timestamp_ = msg_timestamp;
  prefused_message->seq_num_ = seq_num_;
  //单目相机检测阶段
  prefused_message->process_stage_ = ProcessStage::MONOCULAR_CAMERA_DETECTION; 
  prefused_message->sensor_id_ = camera_name;
  prefused_message->frame_ = base::FramePool::Instance().Get();
  prefused_message->frame_->sensor_info = sensor_info_map_[camera_name];
  prefused_message->frame_->timestamp = msg_timestamp
  ...
  prefused_message->frame_->sensor2world_pose = camera2world_trans;
  prefused_message->frame_->objects = camera_frame.tracked_objects; 
    
}
```

相关处理和障碍物属性都添加到`prefused_message`之后，进行消息的发布

```c++
bool send_sensorframe_ret = sensorframe_writer_->Write(prefused_message);
```



## 相机障碍物检测算法总体流程

![](C:\Users\jia_z\Desktop\Apollo_note\image\algorithm.png)

1. 状态预测：对障碍物的属性中使用`KalmanFilter`进行滤波的状态进行预测
2. 障碍物检测：通过神经网络的检测器获取障碍物的2D检测框，障碍物类型，障碍物的尺寸等信息
3. 特征提取：通过检测网络的分支进行障碍物的特征提取，以进行障碍物之间关联
4. 2D关联：利用2D信息对障碍物进行跟踪
5. 2D->3D转换：利用几何约束和先验知识获取障碍物在3D空间中的位置，朝向和尺寸
6. 3D关联：利用3D信息对障碍物进行跟踪




## 主入口

` cyber/mainboard/mainboard.cc/int main(int argc,char** argv){...}` 

> 解析配置参数；初始化cyber环境；由Moudule Controller类创建对象`controller`,调用`controller.Init()` 启动各功能模块；进入Cyber RT消息循环，等待`apollo::cyber::WaitForShutdown();` 返回Main函数，清理资源退出。

`controller.Init()` 调用 `ModuleController::LoadAll()`

`cyber/mainboard/module_controller.cc/bool ModuleController::LoadAll(){...}`

> 循环读取DAG配置文件列表，得到配置文件中所有`dag_conf` ,调用
>
> `bool ModuleController::LoadModule(const std::string& path)`  加载模块功能

`cyber/mainboard/module_controller.cc/ModuleController::LoadModule(&path)`

> 根据路径得到dag文件配置，调用`bool ModuleController::LoadModule(const DagConfig& dag_config)` 实现加载模型。

- component文件结构：
  - 头文件
  - 实现文件
  - 构建文件：BUILD
  - DAG配置文件
  - Launch启动文件
- 实现Component（Compnent组件构建过程）
  - 基于模板类`cyber::Component` 派生各组件类
  - 在派生类(` FusionComponent` )中覆盖虚函数`Init()` 和`Proc()`
  - 使用宏`CYBER_REGISTER_COMPONENT(FusionComponent)` 注册组件类
- 消息接收发送的方式

---------------------------

## 感知模块对象创建过程

apollo中对象的创建大多采用直接法，也有部分对象使用单例模式创建`DECLARE_SINGLETON(AdapterManager)`,还有部分对象采用工厂模式创建。

感知（perception）模块位于命名空间`apollo::perception` 中，创建过程

可能用到了**工厂模式** 动态创建`apollo::perception::FusionCameraDetectionComponent `类对象，首先生成一个与之对应的工厂类，并将其加入到工厂集合类 `std::map` 中 (参考planning模块，是否如此 存疑)

具体参考文件：

`cyber/class_loader/utility/...` 

创建过程位于` bool ModuleController::LoadModule(const DagConfig& dag_config)`中

```c++
bool ModuleController::LoadModule(const DagConfig& dag_config) {
	std::string load_path;
	//...
    class_loader_manager_.LoadLibrary(load_path);

    for (auto& component : module_config.components()) {
      const std::string& class_name = component.class_name();
      std::shared_ptr<ComponentBase> base =
          class_loader_manager_.CreateClassObj<ComponentBase>(class_name);
      if (base == nullptr || !base->Initialize(component.config())) {
        return false;
      }
      component_list_.emplace_back(std::move(base));
    }
	//...
  }
  return true;
}

```

工厂类对象指针找到后使用`classobj = factory->CreateObj();`即可将PerceptionComponent 类对象创建。

-----------------------

## 功能模块构建

> 基于模板类`Component` 派生各组件类
>
> 在派生类` FusionComponent` 中覆盖虚函数`Init()` 和`Proc()`
>
> 使用宏`CYBER_REGISTER_COMPONENT(FusionComponent)` 注册组件类

- 基于模板类`Component` 派生各组件类

模板类Component最多可接受4各模板参数，每个模板参数表示一种输入信息类型。

`cyber/component/component.h`

```c++
template <typename M0 = NullType, typename M1 = NullType,
          typename M2 = NullType, typename M3 = NullType>
class Component : public ComponentBase {...}
```

感知模块各组件类（继承于基类cyber::Component):

`FusionCameraDetectionComponent` ,`FusionComponent`, `LaneDetectionComponent`,`LidarOutputComponent`,`radar_detection_component`,`RecognitionComponent`,`SegmentationComponent`,`TrafficLightPerceptionComponent`.

以`FusionComponent`为例,继承自`cyber::Component<SensorFrameMessage>`，消息参数为`SensorFrameMessage`，这些消息将会在`Proc`函数中周期性接收并处理。

`perception/onborad/component/fusion_component.h`

```c++
class FusionComponent : public cyber::Component<SensorFrameMessage> {
 public:
  FusionComponent() = default;
  ~FusionComponent() = default;
  bool Init() override;
  bool Proc(const std::shared_ptr<SensorFrameMessage>& message) override;

 private:
  bool InitAlgorithmPlugin();
  bool InternalProc(const std::shared_ptr<SensorFrameMessage const>& in_message,
                    std::shared_ptr<PerceptionObstacles> out_message,
                    std::shared_ptr<SensorFrameMessage> viz_message);

 private:
  static std::mutex s_mutex_;
  static uint32_t s_seq_num_;

  std::string fusion_method_;
  std::string fusion_main_sensor_;
  bool object_in_roi_check_ = false;
  double radius_for_roi_object_check_ = 0;

  std::unique_ptr<fusion::ObstacleMultiSensorFusion> fusion_;
  map::HDMapInput* hdmap_input_ = nullptr;
  std::shared_ptr<apollo::cyber::Writer<PerceptionObstacles>> writer_;
  std::shared_ptr<apollo::cyber::Writer<SensorFrameMessage>> inner_writer_;
};
CYBER_REGISTER_COMPONENT(FusionComponent);
```

- 在派生类` FusionComponent` 中覆盖虚函数`Init()` 和`Proc()`

  `Init()` 创建实际对象，创建消息处理回调函数，创建输出器writer

  `Proc()` 周期性接收消息，执行相关处理，并向其他模块输出消息。

- 使用宏`CYBER_REGISTER_COMPONENT(FusionComponent)` 注册组件类，使`Cyber RT` 能够正确创建和加载该类对象。

- BUILD构建文件

  `perception/onboard/component/BUILD` 

  基于`perception_component_inner_camera` 生成 `libperception_component_camera.so`

  基于`perception_component_inner_lidar` 生成 `libperception_component_lidar.so` 

  生成的共享库文件由`Cyber RT` 调度程序mainboard动态加载运行。

- DAG配置文件：`perception/production/dag`

  DAG配置文件是Cyber RT 调度程序`mainboard` 动态加载`perception` 模块的最终配置文件。

  加载命令：`/apollo/cyber/mainboard -d /apollo/modules/perception/production/dag/dag_streamimg_perception.dag`

  ``` c++
  module_config {
    #共享库文件路径
    module_library : "/apollo/bazel-bin/modules/perception/onboard/component/libperception_component_lidar.so"
  
   components {
      #组件名称，mainboard动态加载
      class_name : "SegmentationComponent"
      config {
        #模块名
        name: "Velodyne128Segmentation"
        #绝对路径，配置文件路径
        config_file_path: "/apollo/modules/perception/production/conf/perception/lidar/velodyne128_segmentation_conf.pb.txt"
  
        flag_file_path: "/apollo/modules/perception/production/conf/perception/perception_common.flag"
        #组件proc()函数中使用的消息接收器
        readers {
            channel: "/apollo/sensor/lidar128/compensator/PointCloud2"
          }
      }
    }
  }
  ```

- Launch 启动文件:`modules/perception/prodution/launch/perception_all.launch`

  Launch配置文件使用`Cyber RT` 提供的python工具程序`cyber_launch` 加载`Perception` 模块所需的配置文件，启动命令如下

  `cyber_launch start /apollo/launch/perception_all.launch` 

  典型片段：

  ```c++
  <cyber>  	
  	<module>
          <name>perception</name>
          <dag_conf>/apollo/modules/perception/production/dag
        			/dag_streaming_perception.dag</dag_conf>
          <!-- if not set, use default process -->
          <process_name>perception</process_name>
          <version>1.0.0</version>
      </module>
  <cyber>
  ```

- 接收消息：

  基于`Cyber RT`接收消息分为两种：

  - 虚函数：Proc()中处理指定的消息类型，周期性触发（接收），但最多只能接收4种消息类型（由cyber::Component的模板参数最多只有4个决定），一般用于模块主要输入信息的接收。

  - 直接创建消息接收器，一般用于接收非周期性消息或模块的次要输入消息，例如：

    `modules/perception/onboard/component/fusion_camera_detection_component.cc`

    ```c++
    auto camera_reader = node_->CreateReader(channel_name, camera_callback);
    ```

- 发布消息：

  基于`Cyber RT` 发布消息

  `fusion_camera_detection_component.cc/FusionCameraDetectionComponent::Init() `定义

  ````c++
    sensorframe_writer_ =
        node_->CreateWriter<SensorFrameMessage>(prefused_channel_name_);
  ````

  `FusionCameraDetectionComponent::OnReceiveImage`中发布消息。

  ```c++
  bool send_sensorframe_ret = sensorframe_writer_->Write(prefused_message);
  ```

  

---------------



## 具体算法分析：







## 消息发送接收类型：

`FusionCameraDetectionComponent::Init(){}` 中以模板类的方式开头定义了四个writer:

其内容是`InitConfig()` 时导入的proto消息配置文件`"fusion_camera_detection_component.proto"` 信息 :

``` c
string output_obstacles_channel_name = 10 [default = "/perception/obstacles"];
string prefused_channel_name = 12 [default = "/perception/inner/PrefusedObjects"];
string camera_perception_viz_message_channel_name = 11 [default = "/perception/inner/camera_viz_msg"]; //3d?
string camera_debug_channel_name = 20 [default = "/perception/camera_debug"];
```

``` c
  writer_ =
      node_->CreateWriter<PerceptionObstacles>(output_obstacles_channel_name_); 
  sensorframe_writer_ =
      node_->CreateWriter<SensorFrameMessage>(prefused_channel_name_);
  camera_viz_writer_ = node_->CreateWriter<CameraPerceptionVizMessage>(
      camera_perception_viz_message_channel_name_);
  camera_debug_writer_ =
      node_->CreateWriter<apollo::perception::camera::CameraDebug>(
          camera_debug_channel_name_);
```



### 文件参数初始化的调用关系：

`*.proto` 中包含的相关固定（默认）参数通过编译为对应`package` 命名空间下的类，然后`*.pt` 的文件通过`GetAbsolutePath()` 函数读取，最后通过`cyber::common::GetProtoFromFile()` 函数将`*.pt` 中的参数部分修改`*.proto` 中的默认参数，即得最终的配置参数，然后这些参数复制给对应的类成员属性。

例：

```c++
app::PerceptionParam perception_param_; //对应proto文件生成的类
config_file = GetAbsolutePath(work_root, config_file); //通过路径获取pt文件的路径(obstacle.pt)
cyber::common::GetProtoFromFile(config_file, &perception_param_);//部分修改perception_param中的参数
detector_init_options.gpu_id = perception_param_.gpu_id();//将proto文件参数赋给具体类实例中的成员属性
```



模型参数以及相关配置文件的修改位于：

`modules/perception/production/data/perception/..`中，包含了各个传感器及其对应的功能模块的配置文件



## 目前待解决的问题：

`Cyber RT` 的调度与通信机制，例如Component::Proc()是如何被周期性調用的

camera app中如何被fusion中调用的，消息的传送方向。

### 1. Proto文件解析与C++开发

``` protobuf
syntax = "proto2";

package apollo.perception.camera.yolo; //对应命名空间

message YoloParam {
    optional ModelParam model_param = 1;
    optional NetworkParam net_param = 2;
    optional NMSParam nms_param = 3;
}
```

```c++
#include “yolo.pb.h” //此文件编译后产生
yolo::YoloParam yolo_param_;//调用
```

>  https://www.jianshu.com/p/d2bed3614259

.proto 文件以 package 声明开头，这有助于防止不同项目之间的命名冲突。在 C++ 中，生成的类将放在与包名匹配的 namespace （命名空间）中。

#### 2. TensorRT 模型量化方法



####     caffe blob、TernsorRT、caffe 特征提取的C++接口:



#### 3. 网络推断模型 （inference_factory.cc）的建立和区别(与上面问题存在关联)

> CaffeNet
>
> RTNet
>
> RTNetInt8
>
> PaddleNet

### 4. 视频目标跟踪和目标检测的统一框架：

> Detect to Track and Track to Detect

#### Roi pooling track ?



#### 5. 目标跟踪 匈牙利算法（Hungarian Algorithm）与KM算法（Kuhn-Munkres Algorithm）

> https://blog.csdn.net/NIeson2012/article/details/94472313
>
> https://blog.csdn.net/zziahgf/article/details/85344859
>
> https://blog.csdn.net/xiao__run/article/details/84374959
>
> Deep sort论文需要看一下


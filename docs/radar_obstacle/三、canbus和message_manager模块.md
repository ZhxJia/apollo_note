# 三、canbus和message_manager模块

模块的运行：

在运行了radar的component后，再运行该模块。该模块通过can卡读取数据，根据相应的消息类型进行解析，并发送出来给radar的component收听。

参照README_cn.md（apollo-5.0.0\modules\drivers\radar\conti_radad\README_cn.md）：

\# in docker

```bash
cd /apollo

source scripts/apollo_base.sh
```

\# 启动

```bash
./scripts/conti_radar.sh start
```

\# 停止

```bash
./scripts/conti_radar.sh stop
```

apollo_base.sh会调用相应的launch和dag文件。



模块介绍：

该模块主要涉及conti_radar_canbus_component.cc、conti_radar_canbus_component.h、conti_radar_message_manager.cc、conti_radar_message_manager.h四个文件。

conti_radar_message_manager.h：

声明了ContiRadarMessageManager类，尤其是在类成员中声明了后续需要使用到的can_client。

ContiRadarMessageManager类继承于MessageManager类。

conti_radar_message_manager.cc：

定义了ContiRadarMessageManager类的类函数。

1、ContiRadarMessageManager类在实例化时，会通过构造函数中的AddRecvProtocolData函数，建立protocol_data_map_字典结构。字典的key为message_id，value为指向消息类型实例的指针。

```C++
ContiRadarMessageManager::ContiRadarMessageManager(
    const std::shared_ptr<Writer<ContiRadar>> &writer)
    : conti_radar_writer_(writer) {
  AddRecvProtocolData<RadarState201, true>();
  AddRecvProtocolData<ClusterListStatus600, true>();
  AddRecvProtocolData<ClusterGeneralInfo701, true>();
  AddRecvProtocolData<ClusterQualityInfo702, true>();
  AddRecvProtocolData<ObjectExtendedInfo60D, true>();
  AddRecvProtocolData<ObjectGeneralInfo60B, true>();
  AddRecvProtocolData<ObjectListStatus60A, true>();
  AddRecvProtocolData<ObjectQualityInfo60C, true>();
}
```

2、GetMutableProtocolDataById

根据message_id获得相应的消息类型实例指针

3、set_radar_conf

调用RadarConfig200消息类中的set_radar_conf，将参数radar_conf传给RadarConfig200消息类实例中定义的Radarconf类成员。

```C++
RadarConf radar_conf_;
```

3、Parse函数：

会在从can卡接收到数据后，调用该函数。



conti_radar_message_manager.h：

声明了ContiRadarCanbusComponent类。

conti_radar_message_manager.cc：

定义了ContiRadarMessageManager类的类函数。

1、Init()：

(1)、获取conti_radar_conf配置。

(2)、通过工厂模式实例化can_client，使用的是esd_can_client。

esd_can_client相关文件位于：apollo-5.0.0\modules\drivers\canbus\can_client。

(3)、定义Conti_radar_writer节点，通道是conti_radar_conf种的雷达通道。

(4)、定义pose_reader，并调用callback函数。

(5)、定义消息管理器的指针

(6)、将conti_radar_conf中的radar_conf，传给RadarConfig200消息类实例（在消息管理器中定义的）中定义的Radarconf类成员radar_config_。

(7)、将can_client传给消息管理器，明确消息管理器中所使用的具体的继承后的can_client。

(6)、(7)都是将参数从canbus的定义中，传给message_manager。

(8)、初始化can_receiver_，明确can_receiver所使用的具体的继承后的can_client和具体的消息管理类。

(9)、调用start函数。



2、ConfigureRadar()配置雷达：

(1)、新实例RadarConfig200的类，同样用获取到的conti_radar_conf中的radar_conf。

(2)、将雷达配置的信息，传到将要用来发送的sendermessage类变量中。

sendermessage的数据形式如下：

```C++
 private:
  uint32_t message_id_ = 0;
  ProtocolData<SensorType> *protocol_data_ = nullptr;

  int32_t period_ = 0;
  int32_t curr_period_ = 0;

 private:
  static std::mutex mutex_;
  struct CanFrame can_frame_to_send_;
  struct CanFrame can_frame_to_update_;
```

```C++
struct CanFrame {
  /// Message id
  uint32_t id;
  /// Message length
  uint8_t len;
  /// Message content
  uint8_t data[8];
  /// Time stamp
  struct timeval timestamp;

  /**
   * @brief Constructor
   */
  CanFrame() : id(0), len(0), timestamp{0} {
    std::memset(data, 0, sizeof(data));
  }
```

(3)、用过can_client的sendsingframe函数，将sendermessage中的canframe发往can总线，完成雷达配置。



3、Start()启动流程：

```C++
bool ContiRadarCanbusComponent::Start() {
  // 1. init and start the can card hardware
  if (can_client_->Start() != ErrorCode::OK) {
    return OnError("Failed to start can client");
  }
  AINFO << "Can client is started.";
  if (ConfigureRadar() != ErrorCode::OK) {
    return OnError("Failed to configure radar.");
  }
  AINFO << "The radar is successfully configured.";
  // 2. start receive first then send
  if (can_receiver_.Start() != ErrorCode::OK) {
    return OnError("Failed to start can receiver.");
  }
  AINFO << "Can receiver is started.";

  // last step: publish monitor messages
  monitor_logger_buffer_.INFO("Canbus is started.");

  return true;
}
```

(1)、调用can_client（esd_can_client）的start函数。

调用canOpen函数，启动对can总线的读取。

(2)、配置雷达

(3)、启动can_receiver，通过调用RecvThreadFunc函数实现消息的接收，这里会启动一个异步进程去完成接收。

(4)、RecvThreadFunc通过can_cient_的reeceive函数，进而调用canWrite函数的接收消息。然后通过去解析消息管理器中的Parse函数去解析消息。



4、PoseCallback()：

从pose_reader中，听取pose_msg，并当听到消息时调用该函数。

将所听取的消息，包括线速度和角速度，处理到can的帧信息里，并通过can_client的sendsingframe函数发送到can总线上。
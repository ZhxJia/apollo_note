# 二、Component部分

主要涉及radar_detection_component.h、radar_detection_component.cc两个文件。

radar_detection_component.h：

导入相关头文件、声明RadarDetectionComponent类（继承于 cyber::Component<ContiRadar>）

radar_detection_component.cc：

定义RadarDetectionComponent类函数，也是流程的主体函数。

1、Init()初始化：

调用过程：

(1)、声明并从配置文件中，获取雷达配置

(2)、根据radar_name来获取传感器信息 

(3)、定义writer_

(4)、算法插件初始化，包括：

(4-1)、定义并初始化高精度地图输出

(4-2)、实例、设置和初始化雷达预处理方法

(4-3)、实例、设置和初始化雷达障碍物感知方法

(5)、radar2world_trans_.Init

(6)、radar2novatel_trans_.Init

(7)、localization_subscriber_.Init

建立reader_node：听取里程计通道。

2、Proc()处理：

(1) 、接收message

(2) 、定义out_message，包括：

sensor_id、timestamp、seq_num、type_name、hdmap_、frame_、process_stage

(3) 、InternalProc核心算法处理message，输出到out_message。

(3-1)通过接收到的in_message建立raw_obstacles。

(3-2)设置预处理选项。

(3-3)通过预处理过程，得到正确的corrected_obstacle。

(3-4)分别获取雷达坐标转换信息、车线速度、角速度，来设置感知选项中的检测器选项。

(3-5)通过转换坐标获取相应位置的高精度地图，来设置感知选项中的roi_filter选项。

(3-6)通过感知过程，得到检测物体。

(4) 、通过write发送out_message：

包括时间戳、接收序号、处理阶段、雷达名字、雷达信息、雷达转换信息、检测物体。



具体解释：

1、读取配置： 		

componentbase中的类函数：GetProtoConfig。

在component_base.h中设置config_file_path的路径。尚未设置。

register。

目录位于：

apollo-5.0.0\modules\perception\production\conf\perception\

![img](file:///C:\Users\HUANLO~1\AppData\Local\Temp\ksohtml19956\wps1.jpg) 

 

2、配置中，有一个传感器名。通过传感器类的GetSensorInfor函数，获取传感器信息，如下：

```C++
struct SensorInfo {

	std::string name = "UNKNONW_SENSOR";

	SensorType type = SensorType::UNKNOWN_SENSOR_TYPE;

	SensorOrientation orientation = SensorOrientation::FRONT;

	std::string frame_id = "UNKNOWN_FRAME_ID";

 	void Reset() {

 		name = "UNKNONW_SENSOR";

 		type = SensorType::UNKNOWN_SENSOR_TYPE;

 		orientation = SensorOrientation::FRONT;

 		frame_id = "UNKNOWN_FRAME_ID";

	}

};
```



3、定义写节点，通道为配置中的output_channel_name：

如output_channel_name: "/perception/inner/PrefusedObjects"、

 

4、算法插件的初始化：

这部分初始化了高精度地图、雷达预处理、雷达感知三个部分。

高精度地图：实例化HDMapInput类为hdmap_input，并进行初始化。初始化的过程是通过hdmap_file的地址，读取文件赋给hdmap_变量。	

```C++
hdmap_.reset(new apollo::hdmap::HDMap());
hdmap_file_ = apollo::common::util::StrCat(FLAGS_map_dir, "/base_map.bin");
  AINFO << "hdmap_file_: " << hdmap_file_;
  if (!apollo::cyber::common::PathExists(hdmap_file_)) {
    AERROR << "Failed to find hadmap file: " << hdmap_file_;
    return false;
  }
  if (hdmap_->LoadMapFromFile(hdmap_file_) != 0) {
    AERROR << "Failed to load hadmap file: " << hdmap_file_;
    return false;
  }
```

雷达预处理：根据配置中的雷达预处理方法名字，实例化出具体子类的对象，再用共享指针指向之，然后进行初始化。

雷达感知：根据配置中的雷达感知方法名字，实例化出具体的对象，再用共享指针指向之，然后通过pipeline_name进行初始化。

如：

radar_preprocessor_method: "ContiArsPreprocessor"

radar_perception_method: "RadarObstaclePerception"

radar_pipeline_name: "FrontRadarPipeline"

```C++
radar::BasePreprocessor* preprocessor =
      radar::BasePreprocessorRegisterer::GetInstanceByName(
          preprocessor_method_);
  CHECK_NOTNULL(preprocessor);
  radar_preprocessor_.reset(preprocessor);
  CHECK(radar_preprocessor_->Init()) << "Failed to init radar preprocessor.";
  radar::BaseRadarObstaclePerception* radar_perception =
      radar::BaseRadarObstaclePerceptionRegisterer::GetInstanceByName(
          perception_method_);
  CHECK(radar_perception != nullptr)
      << "No radar obstacle perception named: " << perception_method_;
  radar_perception_.reset(radar_perception);
  CHECK(radar_perception_->Init(pipeline_name_))
```

radar_preprocessor_，_radar_perception_都是父类指针指向子类实例。

初始化时，时根据配置初始化其中的detector，roi_filter，tracker。

配置位于apollo-5.0.0\modules\perception\production\conf\perception\radar\modules，以front_radar_pipeline.config举例。

```
model_configs {
  name: "FrontRadarPipeline"
  version: "1.0.0"
  string_params {
    name: "Detector"
    value: "ContiArsDetector"
  }
  string_params {
    name: "RoiFilter"
    value: "HdmapRadarRoiFilter"
  }
  string_params {
    name: "Tracker"
    value: "ContiArsTracker"
  }
}
```

初始化后，detector、roi_filter、tracker的代码位于：

apollo-5.0.0\modules\perception\production\conf\perception\radar\lib



5、位置订阅器的初始化，通道为：

odometry_channel_name: "/apollo/localization/pose"

radar_name: "radar_front"

通过该通道定义节点，并调用回调函数，又消息msg时，将时间戳和msg压入buffer_queue中。	



6、算法--预处理：

预处理的选项设置为空

```C++
struct PreprocessorOptions {
  // reserved
};
```

由于在预处理初始化的过程中使用了ContiArsPreprocessor，这一预处理方法名。所以在后续的过程中，实际使用的ContiArsPreprocessor这一类型的实例。

相关代码位于：apollo-5.0.0\modules\perception\radar\lib\preprocessor

调用ContiArsPreprocessor类的Preprocess函数，主要处理流程如下：

```C++
bool ContiArsPreprocessor::Preprocess(
    const drivers::ContiRadar& raw_obstacles,
    const PreprocessorOptions& options,
    drivers::ContiRadar* corrected_obstacles) {
  PERCEPTION_PERF_FUNCTION();
  SkipObjects(raw_obstacles, corrected_obstacles);
  ExpandIds(corrected_obstacles);
  CorrectTime(corrected_obstacles);
  return true;
}
```

(1)、SkipObjects：

```C++
 corrected_obstacles->mutable_header()->CopyFrom(raw_obstacles.header());
  double timestamp = raw_obstacles.header().timestamp_sec() - 1e-6;
  for (const auto& contiobs : raw_obstacles.contiobs()) {
    double object_timestamp = contiobs.header().timestamp_sec();
    if (object_timestamp > timestamp &&
        object_timestamp < timestamp + CONTI_ARS_INTERVAL) {
      drivers::ContiRadarObs* obs = corrected_obstacles->add_contiobs();
      *obs = contiobs;
    }
```

获取raw_obstacles的时间戳，并减去1e-6作调整。

之后将raw_obstacles中，时间戳位于处理间隔内的障碍物，加入待处理的corrected_obstacles中。

(2)、ExpandIds

为每个障碍物赋上一个全局的id序号。

(3)、CorrectTime

将时间戳扣去delay_time_，作为最终corrected_obstacles的时间戳。



7、算法--坐标转换：

![avatar](C:\Users\huanloxia\Desktop\radar.png)

通过Getsensor2worldTrans、GetTrans两个函数，获得雷达坐标系到novtel（坐标系）、世界坐标系之间的转换矩阵。其中，坐标系之间的平移、旋转关系，需要通过时间戳去实时地获得。



Getsensor2worldTrans：

输入：timestamp，radar_trans(affine3d)

内部转换矩阵：

转换矩阵1：sensor2novatel_extrinsics(affine3d)

转换关系：trans_sensor2novatel，包含平移矩阵和旋转矩阵。

```C++
struct StampedTransform {
  double timestamp = 0.0;  // in second
  Eigen::Translation3d translation;
  Eigen::Quaterniond rotation;
};
```

转换矩阵1即外参，等于从querytrans函数获取到trans_sensor2novatel，再将他的translation和rotation相乘。



转换矩阵2：novate2world

转换关系：trans_novatel2world

转换矩阵2等于从querytrans函数获取trans_sensor2novatel，再将他的translation和rotation相乘。



输出：radar_trans(sensor2world_trans)

输出等于，将extrinsics矩阵和novatel2world矩阵，两者相乘，得到输出。

即用来转换得到世界坐标系下的位姿(pose)。



GerTrans：

输入：timestamp，radar2novatel_trans(affine3d)

转换关系：transform，包含平移矩阵和旋转矩阵。

输出：等于从querytrans函数获取到transform，再将他的translation和rotation相乘。



Eigen库的矩阵转换关系：

Matrix4f到Affine3f：

Matrix4f m4f_transform;

Eigen::Transform<float, 3, Eigen::Affine> a3f_transform (m4f_transform);

Affine3f到Matrix4f：

Eigen::Transform<float, 3, Eigen::Affine> a3f_transform ;

Matrix4f m4f_transform=a3f_transform.matrix();

则有：

```C++
Eigen::Affine3d radar_trans;
Eigen::Affine3d radar2novatel_trans;
```



8、速度获取：

```C++
(*car_linear_speed)[0] =
      static_cast<float>(loct_ptr->pose().linear_velocity().x());
(*car_linear_speed)[1] =
      static_cast<float>(loct_ptr->pose().linear_velocity().y());
(*car_linear_speed)[2] =
      static_cast<float>(loct_ptr->pose().linear_velocity().z());
(*car_angular_speed)[0] =
      static_cast<float>(loct_ptr->pose().angular_velocity().x());
(*car_angular_speed)[1] =
      static_cast<float>(loct_ptr->pose().angular_velocity().y());
(*car_angular_speed)[2] =
      static_cast<float>(loct_ptr->pose().angular_velocity().z());
```

通过时间戳到msg_buffer中，匹配合适的时间戳，获得线速度和角速度，并分别放入两个vector中后返回。



9、感知选项

感知选项主要包括，检测器选项、Roi滤波选项、跟踪器选项、传感器名字。

```C++
struct RadarPerceptionOptions {
  DetectorOptions detector_options;
  RoiFilterOptions roi_filter_options;
  TrackerOptions track_options
  std::string sensor_name;
};
```

检测器选项主要包括，两个转换矩阵，两个速度向量和高精度地图结构体指针。

```C++
struct DetectorOptions {
  Eigen::Matrix4d* radar2world_pose = nullptr;
  Eigen::Matrix4d* radar2novatel_trans = nullptr;
  Eigen::Vector3f car_linear_speed = Eigen::Vector3f::Zero();
  Eigen::Vector3f car_angular_speed = Eigen::Vector3f::Zero();
  base::HdmapStructPtr roi = nullptr;
};
```

两个转换矩阵通过坐标转换后的矩阵来设置，两个速度向量在上文中提到速度获取函数中来设置。检测器选项中的roi指针未设置。

Roi滤波器选项内容为高精度地图结构体的共享指针。

```C++
struct RoiFilterOptions {
  base::HdmapStructPtr roi = nullptr;
};
using HdmapStructPtr = std::shared_ptr<HdmapStruct>;
```

高精度地图结构体的结构，是由几个点的多边形组成的道路边界、道路多边形、洞多边形、路口多边形。

```C++
struct alignas(16) HdmapStruct {
  std::vector<RoadBoundary> road_boundary;
  std::vector<PointCloud<PointD>> road_polygons;
  std::vector<PointCloud<PointD>> hole_polygons;
  std::vector<PointCloud<PointD>> junction_polygons;
};
```

点的定义如下：

```
template <typename T>
struct Point {
  T x = 0;
  T y = 0;
  T z = 0;
  T intensity = 0;
  typedef T Type;
};
using PointD = Point<double>;
base::PointD position;
position.x = radar_trans(0, 3);
position.y = radar_trans(1, 3);
position.z = radar_trans(2, 3);
```

roi指针的设置，是先通过radar_trans来获取位置position，再根据世界坐标系中的位置，配合雷达向前发射的距离参数，来获取相应区域的高精度地图。同时，高精度的使用与否是通过FLAGS_obs_enable_hdmap_input这一Flag来控制。

```C++
options.roi_filter_options.roi.reset(new base::HdmapStruct());
if (FLAGS_obs_enable_hdmap_input) {
    hdmap_input_->GetRoiHDMapStruct(position, radar_forward_distance_,
                                    options.roi_filter_options.roi);
}
```

首先，根据点和先前距离得到-》车道线id-》车道线-》车道线id得到道路id-》道路-》对于每条道路，有路口则处理路口，没路口则处理道路边界（左右边界及边界上的hole），最终返回路口和道路边界两个数组。

```C++
if (hdmap_->GetRoadBoundaries(point, distance, &road_boundary_vec,
                                &junctions_vec) != 0) {
    AERROR << "Failed to get road boundary, point: " << point.DebugString();
    return false;
```

```C++
int HDMapImpl::GetRoadBoundaries(
    const PointENU& point, double radius,
    std::vector<RoadRoiPtr>* road_boundaries,
    std::vector<JunctionInfoConstPtr>* junctions) const {
  if (road_boundaries == nullptr || junctions == nullptr) {
    AERROR << "the pointer in parameter is null";
    return -1;
  }
  road_boundaries->clear();
  junctions->clear();
  std::set<std::string> junction_id_set;
  std::vector<RoadInfoConstPtr> roads;
  if (GetRoads(point, radius, &roads) != 0) {
    AERROR << "can not get roads in the range.";
    return -1;
  }
  for (const auto& road_ptr : roads) {
    if (road_ptr->has_junction_id()) {
      JunctionInfoConstPtr junction_ptr =
          GetJunctionById(road_ptr->junction_id());
      if (junction_id_set.find(junction_ptr->id().id()) ==
          junction_id_set.end()) {
        junctions->push_back(junction_ptr);
        junction_id_set.insert(junction_ptr->id().id());
      }
    } else {
      RoadRoiPtr road_boundary_ptr(new RoadRoi());
      const std::vector<apollo::hdmap::RoadBoundary>& temp_road_boundaries =
          road_ptr->GetBoundaries();
      road_boundary_ptr->id = road_ptr->id();
      for (const auto& temp_road_boundary : temp_road_boundaries) {
        apollo::hdmap::BoundaryPolygon boundary_polygon =
            temp_road_boundary.outer_polygon();
        for (const auto& edge : boundary_polygon.edge()) {
          if (edge.type() == apollo::hdmap::BoundaryEdge::LEFT_BOUNDARY) {
            for (const auto& s : edge.curve().segment()) {
              for (const auto& p : s.line_segment().point()) {
                road_boundary_ptr->left_boundary.line_points.push_back(p);
              }
            }
          }
          if (edge.type() == apollo::hdmap::BoundaryEdge::RIGHT_BOUNDARY) {
            for (const auto& s : edge.curve().segment()) {
              for (const auto& p : s.line_segment().point()) {
                road_boundary_ptr->right_boundary.line_points.push_back(p);
              }
            }
          }
        }
        if (temp_road_boundary.hole_size() != 0) {
          for (const auto& hole : temp_road_boundary.hole()) {
            PolygonBoundary hole_boundary;
            for (const auto& edge : hole.edge()) {
              if (edge.type() == apollo::hdmap::BoundaryEdge::NORMAL) {
                for (const auto& s : edge.curve().segment()) {
                  for (const auto& p : s.line_segment().point()) {
                    hole_boundary.polygon_points.push_back(p);
                  }
                }
              }
            }
            road_boundary_ptr->holes_boundary.push_back(hole_boundary);
          }
        }
      }  
      road_boundaries->push_back(road_boundary_ptr);
    }
  }
  return 0;
}
```

再从刚才的输出得到，合并得到道路边界和路口，并通过路口过滤道路边界的点。就是转换到后续要使用的HdmapStructPtr中。

```C++
 MergeBoundaryJunction(road_boundary_vec, junctions_vec, &road_boundaries,
                        &(hdmap_struct_ptr->road_polygons),
                        &(hdmap_struct_ptr->junction_polygons));
  // Filter road boundary by junction
 GetRoadBoundaryFilteredByJunctions(road_boundaries,
                                    hdmap_struct_ptr->junction_polygons,
                                    &(hdmap_struct_ptr->road_boundary));
```



10、Perceive

将待处理的物体corrected_obstacles，配合感知选项，处理为最终的radar_objects。该radar_objects会传入out_message中的frame中，通过消息发出。

```C++
std::vector<base::ObjectPtr> radar_objects;
if (!radar_perception_->Perceive(corrected_obstacles, options,
                                 &radar_objects)) {
```

由于在预处理初始化的过程中使用了RadarObstaclePerception，这一预处理方法名。所以在后续的过程中，实际使用的RadarObstaclePerception这一类型的实例。

具体的调用过程中，就是通过该类的detector、roifilter、tracker先后对障碍物进行处理。

detect：

处理corrected_obstacles，得到radar_frame。

对每一个物体，设置其id，track_id，世界坐标系下的坐标，center，anchor_point，世界坐标系下的速度， center_uncertainty，velocity_uncertainty，direction，theta，theta_variance，confidence，motion_state，obstacle_class，size。

roifilter：

对每一个物体进行过滤，需要物体的center需要在之前得到的HdmapStructPtr所表示的roi区域中。

tracker：

使用的是HMMatcher。

考虑 目标 object 和 tracked object 两者的 track_id和几何距离两个因素。

当两者的 track_id 相同，且两者之间距离小于设定值时，认为两者为同一目标，即目标 object 属于该 track。

几何距离 = 两object 的中心点的距离+object 的速度*时间差。



11、out_message：

类型为：sensorfamemessage，内容如下：

错误编码、传感器id、时间戳、处理的消息序号、地图指针、处理阶段、消息帧处理完的结果。

```C++
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

  base::FramePtr frame_;

  ProcessStage process_stage_ = ProcessStage::UNKNOWN_STAGE;
};

```

其中，传感器id、时间戳、处理消息的序号、处理阶段在预处理阶段后填写。

```C++
out_message->timestamp_ = timestamp;
out_message->seq_num_ = seq_num_;
out_message->process_stage_ = ProcessStage::LONG_RANGE_RADAR_DETECTION;
out_message->sensor_id_ = radar_info_.name;
```

Frame信息，在感知算法后填写。

```C++
  out_message->frame_.reset(new base::Frame());
  out_message->frame_->sensor_info = radar_info_;
  out_message->frame_->timestamp = timestamp;
  out_message->frame_->sensor2world_pose = radar_trans;
  out_message->frame_->objects = radar_objects;
```

out_message在消息经过算法处理后，通过writer发出。






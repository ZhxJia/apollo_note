# Apollo转换功能模块Transformer



transformer部分主要进行2D障碍物信息到3D障碍物信息的转换。
该功能模块的相关接口定义`modules/perception/camera/lib/interface/base_obstacle_transformer.h`
![](../../image\transform1.png)



## 1. 初始化

### 1.1相关配置文件

首先在用户接口的配置文件
`modules/perception/production/conf/perception/camera/fusion_camera_detection_component.pb.txt`
中设置`camera_obstacle_perception_conf`的配置路径和配置文件，默认如下：

```protobuf
camera_obstacle_perception_conf_dir : "/apollo/modules/perception/production/conf/perception/camera"
camera_obstacle_perception_conf_file : "obstacle.pt"
```

在上述`obstacle.pt`文件中配置`transformer`功能模块的参数文件路径，默认如下：

```protobuf
transformer_param {
  plugin_param{
    name : "MultiCueObstacleTransformer" //根据此name创建Transformer功能模块对象
    root_dir : "/apollo/modules/perception/production/data/perception/camera/models/multicue_obstacle_transformer"
    config_file : "config.pt"
  }
}
```

根据上述配置文件，`transformer`的配置主要包括：

```c++
min_dimension_val: 0
check_dimension: true
```

一般，各传感器的各功能模块的详细参数配置文件位于`production/data/..`下。

由此，`transformer`功能的外部参数配置已完成，接下来是初始化程序的相关加载。

### 1.2 初始化程序

初始化接口由`FusionCameraDetectionComponent::Init()`进入，通过`InitAlgorithmPlugin`执行`ObstacleCameraPerception::Init()`,然后根据外部配置文件的参数进行`Transformer`功能模块的初始化。
由`obstacle.pt`中的`plugin_param->name`确定`transformer_`指向该功能的实现类为`MultiCueObstacleTransformer`

```c++
bool MultiCueObstacleTransformer::Init(
    const ObstacleTransformerInitOptions &options) {
  std::string transformer_config =
      cyber::common::GetAbsolutePath(options.root_dir, options.conf_file);

  if (!cyber::common::GetProtoFromFile(transformer_config, &multicue_param_)) {
    AERROR << "Read config failed: " << transformer_config;
    return false;
  }
  AINFO << "Load transformer parameters from " << transformer_config
        << " \nmin dimension: " << multicue_param_.min_dimension_val()
        << " \ndo template search: " << multicue_param_.check_dimension();

  // Init object template
  object_template_manager_ = ObjectTemplateManager::Instance();

  return true;
}
```

注意到该初始化，最后通过**单例模式**创建实例`object_template_manager_`。
`object_template_manager_`为类`ObjectTemplateManager`的实例化对象，因为为单例模式，故对象仅能创建一个实例。
其相关初始化流程类似于`Transformer`,前面配置基本一样，其最终的参数配置文件为`../production/data/perception/cammera/common/object_template/object_template.pt`，主要定义了各种目标类型的速度限制及小中大三种尺寸的`HWL`长宽高，作为相关算法的参考，此处不再展开，相关参数配置文件可参考：

```c++
object_template_param {
  plugin_param{
    name : "ObjectTemplate"
    root_dir : "/apollo/modules/perception/production/data/perception/camera/common/object_template/"
    config_file : "object_template.pt"
  }
}
```



## 2. 处理流程

初始化之后，`Transformer`模块算法的执行部分由`FusionCameraDetctionComponent::OnReceiveImage`监听节点调用，然后在其`InternalProc()`中的函数`Perception()`执行`transformer_->Transform(..)`调用`Transformer`模块的算法接口。
在该算法内部主要有三个处理函数。

```c++
  void SetObjMapperOptions(base::ObjectPtr obj, Eigen::Matrix3f camera_k_matrix,
                           int width_image, int height_image,
                           ObjMapperOptions *obj_mapper_options,
                           float *theta_ray);
  int MatchTemplates(base::ObjectSubType sub_type, float *dimension_hwl);
  void FillResults(float object_center[3], float dimension_hwl[3],
                   float rotation_y, Eigen::Affine3d camera2world_pose,
                   float theta_ray, base::ObjectPtr obj);
```

由之前的功能模块，我们得到了通过神经网络检测到的障碍物的2D信息，`Transformer`将根据`detector`得到的检测框的位置`bbox2d`、障碍物的尺寸`dimension_hwl`以及障碍物的朝向`rotation_y`,并以之前初始化的模板尺寸`object_template`作为参考，对每一个检测到的障碍物进行处理，下面介绍具体的功能实现：

### 2.1 SetObjMapperOptions()

```c++
  // @brief: 初始化obj_mapper_options的参数配置
  // @param [in]: detected_object,camera_k_matrix,width_image,height_image
  // @param [in/out]: obj_mapper_options,theta_ray
    void SetObjMapperOptions(base::ObjectPtr obj, Eigen::Matrix3f camera_k_matrix,
                           int width_image, int height_image,
                           ObjMapperOptions *obj_mapper_options,
                           float *theta_ray);
```

该函数主要进行solve3d的前期数据处理，相关处理包括：`prepare 2dbbox,prepare dimension_hwl,prepare rotation_y`
其中主要说明`rotation_y`的处理方式，detector直接得到的角度是`alpha`(相当于图中的$\theta_l$),然后由相机坐标系下物体底部中心`(x,z)`计算得到`theta_ray`(相当于图中$\theta_{ray}$),即可得到车辆的全局方向`rotation_y`。
​																	<img src="../../image\transform3.png" style="zoom: 50%;" />

```c++
  float box_cent_x = (bbox2d[0] + bbox2d[2]) / 2;
  Eigen::Vector3f image_point_low_center(box_cent_x, bbox2d[3], 1); //2d框底边中心
  Eigen::Vector3f point_in_camera =
      static_cast<Eigen::Matrix<float, 3, 1, 0, 3, 1>>(
          camera_k_matrix.inverse() * image_point_low_center);
  *theta_ray =
      static_cast<float>(atan2(point_in_camera.x(), point_in_camera.z()));
  float rotation_y =
      *theta_ray + static_cast<float>(obj->camera_supplement.alpha);//rotation_y即为车辆全局方向
```

最后还通过`MatchTemplates()`函数得到当前检测物体对应最小模板尺寸（也是该类型的模板尺寸在`veh_hwl_`这个容器中的索引起始值），最后将这些处理的信息存储到`obj_mapper_options`这个数据结构中以进行下一步处理。


### 2.2 mapper_->Solve3dBbox() 

```c++
  // @brief: main interface, process
  // @param [in]: obj_mapper_options,obj_center,,dimension_hwl,rotation_y
  // @param [in/out]: obj_mapper_options,theta_ray
  // @notes: center is the bottom-face center ("center" for short)
  bool Solve3dBbox(const ObjMapperOptions &options, float center[3],
                   float hwl[3], float *ry);
```

该函数为2d转3d的主要处理函数，主要处理流程：

- （1）首先，检查车辆类型尺寸的合理性,如下面流程图所示：

  <img src="../../image\transform4.png" style="zoom:75%;" />

- （2）解析3d box的参数,通过投影关系与几何运算得到3d box的`hwl,ry,center`

  ```c++
    // @brief: call 3d solver,获取3d box的参数
    // @param [in/out]: bbox，hwl,ry,center
  bool ObjMapper::Solve3dBboxGivenOneFullBboxDimensionOrientation(
      const float *bbox, const float *hwl, float *ry, float *center)
  ```

  解析3d box中内部的主要处理算法为：

  - `SolveCenterFromNearestVerticalEdge()`:由深度信息反投影得到3d box中心点center 
    首先计算相机中心到障碍物最近一条边的距离：
    	               <img src="../../image\transform6.png" style="zoom: 33%;" />

    然后再加上最近一条边的点到物体中心的距离：

    ![](../../image\transform7.png)

    通过上面的计算得到深度信息后，就可以将2d点反投影回3D空间，得到center。

  - `PostRefineOrientation()`: 将角度$2\pi$分成36等份(每份10°)，将方向角度每次加10°，然后计算由此旋转矩阵组成的投影矩阵将3d box的8个角点投影得到的2d box与检测器得到的2d box计算`IOU`分数，确定最终的转角`ry`
  - `PostRefineZ()`:进一步精细化深度，将`offset`设置为(-1~1)，步长为0.2m,同理改变深度后得到的投影矩阵将3d角点投影分别计算上述的`IOU`分数，确定得分最高的深度z。

- （3）计算转角`yaw`和深度`z`的（协）方差：

  这部分计算估计的`yaw`和`z`的不确定度，估计不确定度的数据来源：

  - yaw：通过之前`PostRefineOrientation`处每10度计算得到的`IOU`分数组成的`ry_score_`计算得到
  - z：设z的变化值为$rz=0.1*z$,然后变化范围(-rz.+rz)且每次步进的值为$2*rz/15$，由此计算得到的`IOU`分数作为原始数据计算得到不确定度`postion_uncertainty_`

通过上述处理最终就得到了3d box的`hwl,rotation,center`,接下来将得到的结果存储到`frame`中，方便之后的算法调用。

### 2.3 FillResults()

```c++
  // @brief: fill back results,将最终处理得到的3D信息存储到frame->detected_objects
  // @param [in]: obj_center,dimension_hwl,rotation_y,camera2world_pose,theta_ray,detected_object
  // @param [in/out]: obj (detected_objects)
  // @notes: 
  void FillResults(float object_center[3], float dimension_hwl[3],
                   float rotation_y, Eigen::Affine3d camera2world_pose,
                   float theta_ray, base::ObjectPtr obj);
```

该函数将对每个obj最终处理得到的3D信息保存`frame->detcted_objecteds`中，以通过`frame`数据提供给其他算法。
添加或更新的信息主要包括：
`obj->camera_supplement.local_center(3)` :center in camera coordinate system
`obj->center(3)` : center of the boundingbox (cx, cy, cz), 世界坐标系
`obj->size(3)`: size = [length, width, height] of boundingbox,length is the size of the main direction
`obj->center_uncertainty(3)`:covariance matrix of the center uncertainty (type:Matrix3f)
`obj->direction[3]`: main direction of the object,由旋转角转换为方向向量，世界坐标系
`obj->theta ` :the yaw angle, theta = 0.0 <=> direction(1, 0, 0) ，与direction对应。
`obj->theta_variance`
`obj->camera_supplement.alpha`: alpha angle from KITTI: Observation angle of object, in [-pi..pi]



## 3. 补充材料

### Transform主要处理函数示意图

![Transform主要处理函数](../../image\transform2.png)

### 相机坐标系到图像坐标系的投影

![](../../image\transform5.png)

### 角点坐标到图像坐标的转换

![](../../image\transform8.png)

### 参考资料

> 1.single view metrology[http://ieeexplore-ieee-org-s.ivpn.hit.edu.cn:1080/document/791253]
> 2.http://www.cvlibs.net/datasets/kitti/setup.php
> 3.https://cloud.tencent.com/developer/article/1418687
>
> 
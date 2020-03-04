# Apollo跟踪器Tracker

`Tracker`功能模块实现对检测目标的跟踪，该功能模块包含`Predict`,`Associate2d`,`Associate3D`等子模块。

该功能模块的接口类定义`modules/perception/camera/lib/interface/base_obstacle_tracker.h`
![跟踪器接口](C:\Users\jia_z\Desktop\Apollo_note\image\tracker1.png)

## 1.初始化

### 1.1 相关配置文件

首先在用户接口配置文件：
`modules/perception/production/conf/perception/camera/fusion_camera_detection_component.pb.txt`
中设置`camera_obstacle_perception_conf`(相机障碍物感知)的配置路径和配置文件，默认如下：

```c++
camera_obstacle_perception_conf_dir : "/apollo/modules/perception/production/conf/perception/camera"
camera_obstacle_perception_conf_file : "obstacle.pt"
```

在上述`obstacle.pt`文件中配置`tracker`功能模块的参数文件路径以及具体实现的功能类名称：

```c++
tracker_param {
  plugin_param{
    name : "OMTObstacleTracker"//功能实现类
    root_dir : "/apollo/modules/perception/production/data/perception/camera/models/omt_obstacle_tracker"
    config_file : "config.pt"
  }
}
```

在上述的`omt_obstacle_tracker/config.pt`中对跟踪模块参数定义`modules/perception/camera/lib/obstacle/tracker/omt/omt.proto`进行修改，主要参数包括`KalmanParam,TargetParam,ReferenceParam,WeightParam,OmtParam`,详细定义参考文件，此处不赘述。

### 1.2初始化程序

初始化接口由`FusionCameraDetectionComponent::Init()`进入，通过`InitAlgorithmPlugin`执行`ObstacleCameraPerception::Init()`,然后根据外部配置文件的参数进行`tracker`功能模块的初始化。
	(1) 首先`tracker_`指向配置文件中功能实现类`OMTObstacleTracker`
	(2)  `OMTObstacleTracker`初始化

```c++
// @brief: OMTObstacleTracker初始化，将外部参数存储到类成员变量中
// @param [in]: 参数配置(load by `omt.proto`)
// @note: 下面是简化说明
bool OMTObstacleTracker::Init(const ObstacleTrackerInitOptions &options) {
    ...
	frame_list_.Init(omt_param_.img_capability()); //存储帧的列表，达到capability开始覆盖
    similar_map_.Init(omt_param_.img_capability(), gpu_id_); //存储detect与target的特征向量相似程度
    reference_.Init(omt_param_.reference(), width_, height_); //用于估计地平面
    kTypeAssociatedCost_.emplace_back(std::vector<float>(n_type, 0)); //关联算法中 类间切换的代价
}
```



---

## 2. 处理流程

### 2.1 tracker_->Predict()

首先执行`Predict`子模块，该子模块主要通过`KalmanFilter`等滤波方法根据建立的系统动态模型(`一般为恒速度模型`)预测各跟踪目标`target`的状态：

```c++
// @brief: image_center(相机坐标系下物体中心的预测)，world_center_const(世界坐标系下物体中心的预测)
// @param [in/out]: frame(保存了检测物体和跟踪目标属性的frame)
// @note: 
void Target::Predict(CameraFrame *frame) {
  ...
  image_center.Predict(delta_t);//jac<to do> 根据图像中心点调用kalman恒定速度模型预测模型
  ...
  world_center_const.Predict(delta_t);
}
```

-----

### 2.2 tracker->Associate2D()

由于关联部分需要用到特征提取的信息，此处插入extrator_->Extract(frame)的说明（在之前detector也有介绍）:

```c++
struct FeatureExtractorLayer {
  std::shared_ptr<inference::Layer<float>> pooling_layer;
  std::shared_ptr<base::Blob<float>> rois_blob;
  std::shared_ptr<base::Blob<float>> top_blob;
};
```

通过上述定义的特征提取层进行特征提取，将最后经过`roipooling`的结果存入`track_feature_blob`中。
2D关联部分用到的各种变量定义比较多，下面先行介绍各种成员变量所表示的含义：

#### 2.2.1 主要成员变量

1. 成员类帧列表:`FrameList` 包装了`CameraFrame`类

   ```c++
     int frame_count_ = 0; //添加帧的数量
     int capability_ = 0;	//容量(14)表示14帧后开始覆盖
     std::vector<CameraFrame *> frames_;
   ```

2. 相似性map:`SimilarMap` 存储blob的嵌套vector

   ```c++
     std::vector<std::vector<std::shared_ptr<base::Blob<float>>>> map_sim;
     int dim;//=omt_param_.img_capability()=14
   ```

3. 相似性计算`BaseSimilar`  实例化：`std::shared_ptr<BaseSimilar> similar_`

   ```c++
   class BaseSimilar {
    public:
     virtual bool Calc(CameraFrame *frame1, CameraFrame *frame2,base::Blob<float> *sim) = 0;
   };
   class CosineSimilar : public BaseSimilar {...};
   class GPUSimilar : public BaseSimilar {...};
   ```

   > 包含两种实现，其中GPUSimilar用到了BLAS(线性代数)库:https://blog.csdn.net/cocoonyang/article/details/58602654?depth_1-utm_source=distribute.pc_relevant.none-task&utm_source=distribute.pc_relevant.none-task

4. `Target`类是跟踪算法中**最重要**的数据结构，其包含了单个跟踪器的所有状态信息，比较重要的有`tracked_objects`包含该跟踪器的轨迹（即该跟踪器的所有曾经跟踪过的目标）
   
```c++
   TrackObjectPtrs tracked_objects; 
```
   
5. `TrackObject`类则封装了跟踪目标`Object`类，用于跟踪的目标的最小封装单位，记录该跟踪目标对应的帧id和传感器名称：

   ```c++
   struct TrackObject {
     PatchIndicator indicator; //保存了传感器的名称和检测目标在帧中的id
     double timestamp;
     //由检测得到的原2dbox　经projected_matrix投影得到障碍物的bbox，将不同的相机的bbox统一到一个尺度
     base::BBox2DF projected_box;
     base::ObjectPtr object;
   };
   ```

6. `PatchIndicator`类包含信息如下：

   ```c++
   int frame_id;//为该检测目标对应的是第几帧(frame id )
   int patch_id;//该检测目标在该帧中的id(object id)
   std::string sensor_name;//记录检测到的传感器名称
   ```

7. `ObstacleReference`类包含了与模板相关的参数,存储后期估计地平面的相关信息。

   ```c++
   // @note: `ref_map_`中存储了`reference_`对应元素的索引,即ref_map_中各个位置存储了索引，
   //         索引对应于`reference _`中的的对应Reference属性
   std::map<std::string, std::vector<Reference>> reference_;
   std::map<std::string, std::vector<std::vector<int>>> ref_map_;
   ```

8. `reference`类，每个目标用于估计地平面所需提供的信息

   ```c++
   struct Reference {
     float area = 0.0f; //bbox面积
     float k = 0.0f; 	 //object.H/box.h
     float ymax = 0.0f; //box底边
   };
   ```

#### 2.2.2 Associate2D算法流程

  1. 计算各帧与当前帧的余弦相似性:（第5步target和det_obj的Apperance的匹配分数会用到）

     ```c++
       //计算各帧与当前帧的余弦相似性
       for (int t = 0; t < frame_list_.Size(); t++) {
         int frame1 = frame_list_[t]->frame_id;
         int frame2 = frame_list_[-1]->frame_id;
         similar_->Calc(frame_list_[frame1], frame_list_[frame2],
                        similar_map_.get(frame1, frame2).get());
       }
     ```

  2. 若帧容量已满(frame_count_ >= capability_)，则移除时间target中时间最久远的检测目标。

     ```c++
     target.RemoveOld(frame_list_.OldestFrameId());
     ```

  3. 根据当前帧的检测到的物体，封装为用于检测的目标类型(`TrackObject`)  `det_obj->track_obj`

     ```c++
       for(...){
           ...
       	ProjectBox(frame->detected_objects[i]->camera_supplement.box,
                    frame->project_matrix, &(track_ptr->projected_box));//图像2dbox投影到障碍物坐标系
       	track_objects.push_back(track_ptr);
       }
     ```

  4. 校正当前帧检测目标的三维尺寸(主要是高度)，执行了四种校正方式：分别为模板，参考地平面，标定，历史信息

      ```c++
       // @note: reference_中地平面的检测与更新位于Associate3D中
       reference_.CorrectSize(frame); 
      ```

  5. 生成假设

     ```c++
     // @brief: 评估新检测目标track_objects与targets的相似性
     // @param [in]: track_objects :该帧检测器新检测到的目标
     // @param [in/out]: 
     // output:target匹配第(frame_id)帧中的第几个检测物(track_object),
     //        并将该track_object加入到target的tracked_objects中
     GenerateHypothesis(track_objects);
     ```

  6. 创建新跟踪目标(target)

     ```c++
     int new_count = CreateNewTarget(track_objects);
     ```

     由`track_objects`创建新的跟踪目标的条件有：

     - 首先该det_obj与现有的target的匹配分数很小，即该det_obj与现有target匹配失败
     - 该det_obj的box需要是有效的(bbox宽高大于20同时小于图像宽高)
     - 该det_obj的box矩形没有被某个target的bbox覆盖 ,此处target的bbox指tracked_objects中最近检测的bbox
     - 在上述条件都满足的前提下，需满足det_obj的高度大于最小模板的高度,也可以是未知类型(此时高度的模板未知)
       

  7. 超过存活周期(lost_age>5)的target,否则执行(即正常跟踪的target)下列方法，通过`latest_object`进行相应更新.并调用对应的`滤波方法`进行预测

     ```c++
     // @brief: 对之前Predict的状态根据新的测量值进行修正
     // @note: 2dbox的w,h通过一阶低通滤波器，center通过kalman filter
     target.UpdateType(frame);
     target.Update2D(frame);
     ```

  8. 在Association之后通过IOU合并重复的targets(可能是不同相机得到的)

     ```c++
     CombineDuplicateTargets();
     ```

     - 对目前的targets_两两之间计算其各自含有的tarcked_objects之间的IOU以及box宽和高的差异，计算得到score (此处计算的tracked_object要求他们的时间戳之差小于0.05，同时来自不同传感器)

       ```c++
       score += common::CalculateIOUBBox(box1, box2); 
       ...
       score -= std::abs((rect1.width - rect2.width) *
                                     (rect1.height - rect2.height) /
                                     (rect1.width * rect1.height));
       ```

       将最终平均得分作为这两个`target`之间的相似程度，并将结果保存到score_list中(包括了两个target的索引及其得分)

     - 按照得分从大到小排序依次匹配，可以看出这一步骤与`OMTObstacleTracker::GenerateHypothesis()`相似不同的是我们这里要删除匹配成功的两个target中`id`大的那个,并将删除的那个target_del中的tracked_obj转移到target_save中。

       ```c++
           if (targets_[pair.target].id > targets_[pair.object].id) {
             index1 = pair.object;
             index2 = pair.target;
           }
           Target &target_save = targets_[index1];
           Target &target_del = targets_[index2];
           for (int i = 0; i < target_del.Size(); i++) {
             // no need to change track_id of all objects in target_del
             target_save.Add(target_del[i]);
           }
       ```

       并将target_save(target的引用)中的tracked_objects按照帧id(frame id)由小到大排序，并更新lastest_object。

       ```c++
           std::sort(
               target_save.tracked_objects.begin(), target_save.tracked_objects.end(),
               [](const TrackObjectPtr object1, const TrackObjectPtr object2) -> bool {
                 return object1->indicator.frame_id < object2->indicator.frame_id;
               });//将targe_save中的tracked_objects按照帧的id由小到大排序
           target_save.latest_object = target_save.get_object(-1);
       ```

       然后将`target_del`中的tracked_objects给清零，最后调用`ClearTargets()`即可将`targets_`多余的target清除掉(将target从后往前填空)

  9. 对经过滤波处理的box(单位：米)映射回图像坐标系（像素）

-----

### 2.3 tracker_->Associate3D(frame)

**主要算法流程如下：**

#### 2.3.1 更新reference

`reference`包含的数据结构

```c++
struct Reference {
  float area = 0.0f; //box的面积
  float k = 0.0f;    //object.H/box.h
  float ymax = 0.0f; //box的bottom right
};//可作为参考的Target（CAR,VAN）所对应的属性
```

```c++
// @brief: 更新与检测地平面
// @param [in]: frame(包含当前帧的检测物体的相关信息),`targets_`（当前所有跟踪器）
// @param [in/out]: 
reference_.UpdateReference(frame,targets_);
```

​	对于每个target中最近检测到的物体，如果是可以参考的类型(CAR,VAN)，同时其box的高度大于50，box的底边位置大于内参矩阵中的c_y(即box尽可能位于图像底部),此时该target可以被参考。将该box的底边中心点也进行下采样(y_discrete=y/25,x_discrete=x/25),这样可以与`ref_map`的尺寸对应。
​    如果此中心点所对应的`ref_map`为0(即该点对应的参考图第一次被使用)，将其对应的Reference信息push到数组`reference_`中存储，并将`ref_map`中对应位置置为当前`refs`存储的元素数(即将refs中对应的索引存储到ref_map的对应位置中);否则，若该点对应的ref_map大于零(表示该点已经存在某个物体，其中存储的值为当时的`refs`中对应的索引)，同时此时box的area大于之前存在的reference的area，则将属性进行替换。

```c++
r.area = box.Area();
r.ymax = y;
r.k = obj->size[2] / (y - box.ymin); // H/h
```

- **通过reference列表检测Ground**	
  	首先对于目前`refs`存储的所有`reference`，将其对应的box底边位置（y_max）以及深度倒数`1/z_ref`存储到`vd_samples`中,当reference的数量大于`min_nr_samples=6`时，可进行Ground的检测(需要在线标定的相机Pitch角度和相机离地平面的高度)

  ```c++
      ground_estimator.DetetGround(
          frame->calibration_service->QueryPitchAngle(),
          frame->calibration_service->QueryCameraToGroundHeight(),
          vd_samples.data(), count_samples);//vd_samples.data()表示指向vd_samples中数组第一个元素的指针
  ```

  - - 通过一致性采样和最小二乘检测拟合地平面方程 $a*y+b*disp+c=0$

      ```c++
      // @brief: 通过一致性采样获取内点
      // @param [in]： vs保存了y_max,ds保存了深度倒数disp构成采样点
      // @param [in/out]: inliers(内点索引)，nr_inliers(内点的个数)
      common::RobustBinaryFitRansac<float, 1, 1, 2, 2,
                                           GroundHypoGenFunc<float>,
                                           GroundFittingCostFunc<float>, nullptr>(
                vs, ds, count_vd, p, &nr_inliers, inliers, kThresInlier, false, true,
                0.99f, kMinInlierRatio)
      ```

      ```c++
      // @brief: 根据一致性随机采样得到的内点拟合平面方程
      // @output: l_best(最优平面参数)
      common::ILineFit2dTotalLeastSquare(vd, l_best, count); //再根据这些内点进行最小二乘
      ```

  #### 2.3.2 移除异常移动值

   首先根据其位置信息，移除异常移动的target，并将这些target的最新检测目标创建新的target,同时更新这些新创建的target的2D和类型信息(通过相应的滤波器更新当前状态)

  ```c++
    for (auto &target : targets_) {
      if (target.isLost() || target.Size() == 1) {
        continue;
      }
      auto obj = target[-1]->object;
      if (obj->type != base::ObjectType::UNKNOWN_UNMOVABLE) {
        Eigen::VectorXd x = target.world_center.get_state();
        double move = sqr(x[0] - obj->center[0]) + sqr(x[1] - obj->center[1]);//target最新的object世界坐标系下中心的x,y的移动
        float obj_2_car_x = obj->camera_supplement.local_center[0];
        float obj_2_car_y = obj->camera_supplement.local_center[2];
        float dis = obj_2_car_x * obj_2_car_x + obj_2_car_y * obj_2_car_y;
        if (move > sqr(omt_param_.abnormal_movement()) * dis) {             //0.3*dis
          AINFO << "Target " << target.id << " is removed for abnormal movement";
          track_objects.push_back(target.latest_object);//将移除的target的最新检测目标暂存
          target.Clear(); //异常移动 移除之前得到tracked_objects
        }
      }
    }
  ```

  #### 2.3.3 更新3D信息状态

  ```c++
  // @brief: 根据当前测量值，更新障碍物3D状态信息
  // @param [in]： frame(保存了当前检测得到的3D信息)
  // @output: 物体的velocity,direction,world_center(世界坐标系下的位置),world_center_uncertainty
  target.Update3D(frame);
  ```

  > FirstOrderRCLowPassFilter direction
  >
  > MeanFilter world_center_for_unmovable;
  >
  > KalmanFilterConstVelocity world_center;
  >
  > KalmanFilterConstState<2> world_center_const; // constant position kalman state
  >
  > MeanFilter world_velocity;
  >
  > MeanFilter displacement_theta;//位移朝向

最后将跟踪器target中最新的跟踪目标的属性存储到`frame->tracked_objects`:

```c++
frame->tracked_objects.push_back(target[-1]->object);
```

至此，跟踪部分整体流程完成。

## 参考资料

**ground4与ground3之间的转化关系**:
$Ax+By+Cz+D=0 -> a*y+b*disp+c=0$

<img src="C:\Users\jia_z\Desktop\Apollo_note\image\tracker2.jpg" style="zoom: 33%;" />



**基于RANSAC随机一致性采样的鲁棒方法**

通过ransac算法不断的对平面方程参数进行估算，先介绍一下RANSAC算法:

RANSAC通过反复选取数据中的一组随机子集来达成目标，被选取的子集被假设为局内点，并通过下属方法进行验证：
	(1) 有一个模型适用于假设的局内点，及所有未知参数都能从假设的局内点中计算得出（**拟合模型**）
	(2) 用（1）中得到的模型去测试其他的数据，如果某个点适用于估计的模型(距离小于阈值)，认为它也是局内点。
	(3) 如果有足够多的点被归类为假设的局内点，则估计的模型就足够合理。
	(4) 然后，用所有假设的局内点重新估计模型，因为它仅仅被初始的假设局内点估计过。
	(5) 最后根据估计局内点与模型的错误率来评估模型。

这个过程重复执行固定次数，每次产生的模型要么因为局内点太少而被舍弃，要么因为它比现有的模型更好而被采用。

相比于最小二乘采用了所有点，RANSAC仅采用局内点进行模型的计算，局外点并不影响模型效果。

对于平面的拟合基本步骤如下:
	(1) 随机取样：随机抽取数据作为样本
	(2) 拟合模型：根据样本获取模型参数M
	(3) 判断距离: 判断所有数据点到模型的距离，将距离小于阈值的加入局内点，记录此时模型的局内点数
	(4) 判断：局内点数目>阈值，则利用此时的局内点重新估计模型，重复3-5次，将得到的局内点数最多的模型即为当前					的最佳模型；局内点数目<阈值,   则跳出
	(5) 跳到步骤(1)，循环N次
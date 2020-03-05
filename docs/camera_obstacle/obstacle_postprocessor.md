# Apollo障碍物后处理模块postprocessor

`postprocessor`功能模块将`Transform`功能模块得到的3D信息进一步优化，该功能模块接口：
`modules/perception/camera/lib/interface/base_obstacle_postprocessor.h`

```c++
class BaseObstaclePostprocessor {
  public:
  BaseObstaclePostprocessor() = default;

  virtual ~BaseObstaclePostprocessor() = default;

  virtual bool Init(const ObstaclePostprocessorInitOptions& options =
                        ObstaclePostprocessorInitOptions()) = 0;

  // @brief: refine 3D location of detected obstacles.
  // @param [in]: options
  // @param [in/out]: frame
  // 3D information of obstacle should be filled, required.
  virtual bool Process(const ObstaclePostprocessorOptions& options,
                       CameraFrame* frame) = 0;

  virtual std::string Name() const = 0;

  BaseObstaclePostprocessor(const BaseObstaclePostprocessor&) = delete;
  BaseObstaclePostprocessor& operator=(const BaseObstaclePostprocessor&) =
      delete;
};  // class BaseObstaclePostprocessor
```



## 1. 初始化

###  1.1相关配置文件

首先在用户接口的配置文件
`modules/perception/production/conf/perception/camera/fusion_camera_detection_component.pb.txt`
中设置`camera_obstacle_perception_conf`的配置路径和配置文件，默认如下：

```c++
camera_obstacle_perception_conf_dir : "/apollo/modules/perception/production/conf/perception/camera"
camera_obstacle_perception_conf_file : "obstacle.pt"
```

在上述`obstacle.pt`文件中配置`postprocessor`功能模块的参数文件路径，默认如下：

```c++
postprocessor_param {
  plugin_param{
    name : "LocationRefinerObstaclePostprocessor"
    root_dir : "/apollo/modules/perception/production/data/perception/camera/models/location_refiner_obstacle_postprocessor"
    config_file : "config.pt"
  }
}
```

根据外部参数配置定义文件` ../perception/lib/obstacle/postprocessor/location_refiner/location_refiner.proto`，按照上述`config.pt`更改默认配置如下

```c++
min_dist_to_camera: 30.0
roi_h2bottom_scale: 0.5
```

由此，`postprocessor`的外部参数配置完成，接下来是初始化程序的加载。

### 1.2 初始化程序

初始化接口由`FusionCameraDetectionComponent::Init()`进入，通过`InitAlgorithmPlugin`执行`ObstacleCameraPerception::Init()`,然后根据外部配置文件的参数进行`postprocessor`功能模块的初始化。
根据上述`obstacle.pt`中的`plugin_param`确定`obstacle_postprocessor_`指向的功能实现类名称为
`LocationRefinerObstaclePostprocessor`,后处理部分的初始化较为简单，主要是加载了上面`config.pt`中的参数。

```c++
obstacle_postprocessor_->Init(obstacle_postprocessor_init_options)
```



-----

## 2. 处理流程

初始化后，`postprocessor`模块算法的执行部分由`FusionCameraDetctionComponent::OnReceiveImage`监听节点调用，然后在其`InternalProc()`中的函数`Perception()`执行`postprocessor`模块的算法接口:
`obstacle_postprocessor_-Process(obstacle_postprocessor_options,frame)`

 1. 首先需要通过`OnCalibration`获取地平面方程：

    ```c++
    // @brief: 通过在线标定service获取地平面
    // @param [in/out]: plane
    // @note: plane[4]通过四个参量表示平面ax+by+cz+d=0
    frame->calibration_service->QueryGroundPlaneInCameraFrame(&plane)
    ```

2. 后处理内部功能函数初始化

   ```c++
   // @brief: 内部具体功能实现类的初始化
   // @param [in]: k_mat（内参矩阵），width_image,height_image(图像的高和宽)
   // @note: 此处为内部处理函数的功能类`ObjPostProcessor`的初始化
   postprocessor_->Init(k_mat, width_image, height_image);
   ```

3. 判断`frame->detected_objects`检测物体列表中各物体是否位于roi内

   ```c++
   // @brief: 判断检测物体的底边中心是否位于roi内
   // @param [in]: bottom_center(box底边中心)，图像宽高，内参矩阵，h_down
   // @note: 未在roi内的物体不进行后处理
   float h_down = (static_cast<float>(height_image) - k_mat[5]) *
       		   location_refiner_param_.roi_h2bottom_scale();
   bool is_in_rule_roi =
           is_in_roi(bottom_center, static_cast<float>(width_image),
                     static_cast<float>(height_image), k_mat[5], h_down);
   ```

   ROI区域设置大体如下图红色区域：

   ![](../../image\postprocessor.png)

4. 经过过滤之后的检测物体进行进一步处理，将检测到的物体属性拷贝到`ObjPostProcessionOptions`中进行后处理。
   其所存储的信息如下：

   ```c++
   struct ObjPostProcessorOptions {
     // ground + line segments
     float plane[4] = {0}; 
     std::vector<LineSegment2D<float>> line_segs = {}; //存储由box左上右下两个点组成的线段line_seg
     bool check_lowerbound = true;
   
     // disparity
     float *disp_map = nullptr;
     float baseline = 0.0f;
   
     float hwl[3] = {0};
     float bbox[4] = {0};
     float ry = 0.0f;
   };
   ```

   这一步需要注意的是将检测物体的三维中心下降到地平面上，方便进一步处理：

   ```c++
   // changed to touching-ground center
   object_center[1] += dimension_hwl[0] / 2;
   ```

   > 此处需要注意的是之前`Transformer`部分通过将车辆坐标系下的8个角点投影回图像中,8个角点的坐标中心亦是位于底面，在最终写入frame的时候将`object_center-=dimension_hwl[0] / 2;`变为真正物体中心。
   >
   > ```c++
   > void MultiCueObstacleTransformer::FillResults(
   >     float object_center[3], float dimension_hwl[3], float rotation_y,
   >     Eigen::Affine3d camera2world_pose, float theta_ray, base::ObjectPtr obj) {
   >   if (obj == nullptr) {
   >     return;
   >   }
   >   object_center[1] -= dimension_hwl[0] / 2;
   >   ...
   > ```

5. 通过地平面约束进行障碍物后处理,主要处理函数

   ```c++
   // @brief: 通过地平面约束进行障碍物属性的后处理
   // @param [in]: obj_postprocessor_options，object_center(相机坐标系下物体中心)，hwl,rotation_y
   // @param [in/out]: object_center
   // @note: 
   postprocessor_->PostProcessObjWithGround(
           obj_postprocessor_options, object_center, dimension_hwl, &rotation_y);
   ```

   该函数内部根据地平面通过两种方式进行障碍物后处理：

   **5.1 soft constraints**

   ```c++
   // @brief: 通过地平面约束调整障碍物中心点坐标
   // @param [in]: bbox,hwl,ry,plane,center(相机坐标系下物体中心)
   // @param [in/out]: center
   // @note: 
   bool ObjPostProcessor::AdjustCenterWithGround(const float *bbox,
                                                 const float *hwl, float ry,
                                                 const float *plane,
                                                 float *center) const {
   ```

   正常2d投影点想要反投影回3D空间需要已知深度信息，而此处显然并未提供深度信息，而是通过地面中心点在图像中的投影点逆投影回3D空间则一定处于地平面这个约束来进行反向投影。

   由内参所表示的反投影关系：(x,y)->(X,Y,Z)
   $$
     X=\frac{x-c_x}{f_x}*Z=umcx*Z\\
     Y=\frac{y-c_y}{f_y}*Z=vmcy*Z\\
   $$
     若X,Y,Z位于平面AX+BY+CZ+D=0内，则
   $$
   A*umcx*Z+B*vmcy*Z+C*Z+D=0\\
     A*umcx+B*vmcy+C=-\frac{D}{Z}
   $$

   ```c++
   // @brief: 通过地平面约束将图像中的点x反投影回3d空间点center_test
   // @param [in]: x(2d图像中的物体中心点),k_mat_(内参矩阵),plane(地平面方程)
   // @param [in/out]: center_test(位于地面的中心点)
   // @note: 
   bool in_front = common::IBackprojectPlaneIntersectionCanonical(
           x, k_mat_, plane, center_test);
   ```

   此处，进行center的校正于`Transformer`模块中`center`的更新类似，不过此时依据为物体中心点反投影应位于地平面中。

   **5.2 hard constraints** 

   ```c++
   // @brief: 通过地平面上的线段LineSegment进行优化
   // @param [in]: line_seg_limits(left top -> right bottom)，
   // @param [in/out]: center
   // @note: 
     bool PostRefineCenterWithGroundBoundary(
         const float *bbox, const float *hwl, float ry, const float *plane,
         const std::vector<LineSegment2D<float>> &line_seg_limits, float *center,
         bool check_lowerbound) const;
   ```

   其内部具体又分两步处理：
   5.2.1 GetDepthXPair()

   ```c++
   // @brief: 获得物体中心点在深度z(米)和图像投影点x(像素)的pair
   // @param [in]: bbox(2d),center(3d)
   // @param [in/out]: depth_pts[4](4个底面角点在相机坐标系下深度z),
   // x_pts[4](4底面角点投影到图像坐标系下的x坐标)，pts_c[12](4个底面角点在相机坐标系下的坐标)
   // @note: 
   int GetDepthXPair(const float *bbox, const float *hwl, const float *center,
                       float ry, float *depth_pts, int *x_pts,
                       float *pts_c = nullptr) const;
   ```

   5.2.2 通过地平面中线段调整物体中心点坐标

   ```c++
   // @brief: 通过地平面上的线段获取物体中心点坐标x,z的调整量
   // @param [in]: ls(每个box的线段top left -> bottom right)
   // @param [in/out]: dx_dz
   // @note: ratio_x_over_z = center[0] * common::IRec(center[2]); // x/z
   void GetDxDzForCenterFromGroundLineSeg(const LineSegment2D<T> &ls,
                                          const T *plane, const T *pts_c,
                                          const T *k_mat, int width, int height,
                                          T ratio_x_over_z, T *dx_dz,
                                          bool check_lowerbound = true) { //lineSegment -> 线段
   ```

   注：此函数详细功能实现的理解目前还存在一定的问题，**留坑待填。**



## 参考资料



基于车载单目图像的3维地平面估计_向文辉

![](C:\Users\jia_z\Desktop\Apollo_note\image\postprocessor3.png)

![](C:\Users\jia_z\Desktop\Apollo_note\image\postprocessor2.png)




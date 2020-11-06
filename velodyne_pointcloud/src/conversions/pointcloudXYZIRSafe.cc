#include <velodyne_pointcloud/pointcloudXYZIRSafe.h>

namespace velodyne_pointcloud
{

  PointcloudXYZIRSafe::PointcloudXYZIRSafe(
    const double max_range, const double min_range,
    const std::string& target_frame, const std::string& fixed_frame,
    const unsigned int scans_per_block, boost::shared_ptr<tf::TransformListener> tf_ptr)
    : DataContainerBase(
        max_range, min_range, target_frame, fixed_frame,
        0, 1, true, scans_per_block, tf_ptr, 6,
        "x", 1, sensor_msgs::PointField::FLOAT32,
        "y", 1, sensor_msgs::PointField::FLOAT32,
        "z", 1, sensor_msgs::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::PointField::FLOAT32,
        "ring", 1, sensor_msgs::PointField::UINT16,
        "time", 1, sensor_msgs::PointField::FLOAT32),
        iter_x(cloud, "x"), iter_y(cloud, "y"), iter_z(cloud, "z"),
        iter_ring(cloud, "ring"), iter_intensity(cloud, "intensity"), iter_time(cloud, "time"),
        init(false),img()//16,36000,CV_16UC1,0)
  {
    //cv::Mat(rows,cols,CV_32F,0);
    img = cv::Mat::zeros(16,36000, CV_16UC1);
//          kernel = cv::Mat::ones(config->kernel_size,config->kernel_size,CV_32F )/ (float)(pow(config->kernel_size,2));

  };
  
  void PointcloudXYZIRSafe::finish(){
    sensor_msgs::Image out_msg;
    cv_bridge::CvImage img_bridge;
    std_msgs::Header header;
    cv::Mat tmp(img);
    ROS_INFO_STREAM("IN FINISH");

    try{
      //these lines are just for testing rotating image
      //`cv::Mat rot=cv::getRotationMatrix2D(cv::Point2f(0,0), 3.1416, 1.0);
      //cv::warpAffine(frame,frame, rot, frame.size());
      //if (rotate){
      //  cv::rotate(frame,frame,1);
      //}

      tmp.convertTo(tmp, CV_16UC1);

      img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO16, tmp);//realsense
      img_bridge.toImageMsg(out_msg); // from cv_bridge to sensor_msgs::Image
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR("DID NOT WORK IDIot");
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    img_pub_.publish(out_msg);
    ROS_WARN("Theoretically published");
  }

  void PointcloudXYZIRSafe::setup(const velodyne_msgs::VelodyneScan::ConstPtr& scan_msg){
    DataContainerBase::setup(scan_msg);
    iter_x = sensor_msgs::PointCloud2Iterator<float>(cloud, "x");
    iter_y = sensor_msgs::PointCloud2Iterator<float>(cloud, "y");
    iter_z = sensor_msgs::PointCloud2Iterator<float>(cloud, "z");
    iter_intensity = sensor_msgs::PointCloud2Iterator<float>(cloud, "intensity");
    iter_ring = sensor_msgs::PointCloud2Iterator<uint16_t >(cloud, "ring");
    iter_time = sensor_msgs::PointCloud2Iterator<float >(cloud, "time");

    if (!haspublish && init){ //if in the previous pointcloud nothing found then report free
      std_msgs::Bool b;
      b.data = false;
      safe_pub_.publish(b);
    }

    haspublish = false;
    start = std::chrono::high_resolution_clock::now();//std::chrono::system_clock::now();//std::clock();

    if (init)
    {
      return;
    }
    init = true;

    ros::NodeHandle nh("~");

    srv_ = boost::make_shared <dynamic_reconfigure::Server<velodyne_pointcloud::
      SafeNodeConfig> > (nh);
    dynamic_reconfigure::Server<velodyne_pointcloud::SafeNodeConfig>::
      CallbackType f;
    f = boost::bind (&PointcloudXYZIRSafe::callback, this, _1, _2);
    srv_->setCallback (f);

    safe_pub_ = nh.advertise<std_msgs::Bool>("safe_signal",1);
    img_pub_ = nh.advertise<sensor_msgs::Image>("velodyne_image",1);

  }

  void PointcloudXYZIRSafe::callback(velodyne_pointcloud::SafeNodeConfig &config,
                uint32_t level){
    boost::mutex::scoped_lock ltk(mtx);

    min_x_ = config.min_x;
    min_y_ = config.min_y;
    min_z_ = config.min_z;
    max_x_ = config.max_x;
    max_y_ = config.max_y;
    max_z_ = config.max_z;
    publish_rate = (1.0/config.publish_freq)*1000000000;//nanoseconds;
    ROS_INFO_STREAM("DEPRECATED RATE -> IGNORED IN SAFE "<< publish_rate);
  }

  void PointcloudXYZIRSafe::newLine()
  {}

  void PointcloudXYZIRSafe::addPixel(uint16_t azimuth, float distance, float ring){
    //ROS_INFO_STREAM(azimuth << " RING "<< ring);
    //img->at<uint16_t>(azimuth+ring*img->rows) = distance; 
    //ROS_ERROR("DONE");
  }

  void PointcloudXYZIRSafe::addPoint(float x, float y, float z, uint16_t ring, const uint16_t azimuth, float distance, float intensity, float time)
  {

    addPixel(azimuth, distance,ring);

    if(!pointInRange(distance)) return;

    // convert polar coordinates to Euclidean XYZ

    if(config_.transform)
      transformPoint(x, y, z);

    //std::cout << (x < max_x_) << (x > min_x_) << (z > min_z_) << (z < max_z_) << (y > min_y_) << (y < max_y_)<< std::endl;

    if (x < max_x_ && x > min_x_ && z > min_z_ && z < max_z_ && y > min_y_ && y < max_y_){
      ROS_DEBUG_STREAM("UNSAFE " << x << " : "<< y << " : " << z);

      auto etime = std::chrono::high_resolution_clock::now();
      double elapsed_time =  std::chrono::duration_cast<std::chrono::nanoseconds>(etime - start).count();

      if (!haspublish){//elapsed_time > publish_rate){ //freq to high
        //std::cout << elapsed_time << " : " << publish_rate << std::endl;
        std_msgs::Bool b;
        b.data = true;
        safe_pub_.publish(b);
        start = etime;
        haspublish = true;
      }
    }

    *iter_x = x;
    *iter_y = y;
    *iter_z = z;
    *iter_ring = ring;
    *iter_intensity = intensity;
    *iter_time = time;

    ++cloud.width;
    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_ring;
    ++iter_intensity;
    ++iter_time;
  }
}

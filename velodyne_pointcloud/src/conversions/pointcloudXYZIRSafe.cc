#include <velodyne_pointcloud/pointcloudXYZIRSafe.h>
#include <math.h>

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
    //Original
    //img = cv::Mat::zeros(16,36000, CV_16UC1);
    //img = cv::Mat::zeros(320,1800, CV_16UC1);
    //Small Size to int8
    //img = cv::Mat::zeros(16,36000, CV_8UC1);
    //Resize Image for testing
    img = cv::Mat::zeros(16,360, CV_8UC1);

//          kernel = cv::Mat::ones(config->kernel_size,config->kernel_size,CV_32F )/ (float)(pow(config->kernel_size,2));

  };

  void PointcloudXYZIRSafe::finish(){
    sensor_msgs::Image out_msg;
    cv_bridge::CvImage img_bridge;
    std_msgs::Header header;
    ROS_INFO_STREAM("IN FINISH");

    cv::Mat tmp(img);

    try{
      //these lines are just for testing rotating image
      //`cv::Mat rot=cv::getRotationMatrix2D(cv::Point2f(0,0), 3.1416, 1.0);
      //cv::warpAffine(frame,frame, rot, frame.size());
      //if (rotate){
      //  cv::rotate(frame,frame,1);
            //}

      //tmp.convertTo(tmp, CV_16UC1);

      img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, tmp);//realsense
      img_bridge.toImageMsg(out_msg); // from cv_bridge to sensor_msgs::Image
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR("DID NOT WORK IDIot");
      ROS_ERROR("cv_bridge exception: %s", e.what());
      img = cv::Mat::zeros(16,360, CV_8UC1);
      ROS_ERROR_STREAM(*(iter_x));
      ROS_ERROR_STREAM(*(iter_y));
      return;
    }
    ROS_ERROR("should work");
      ROS_ERROR_STREAM(sizeof(iter_x)/sizeof(float));
      ROS_ERROR_STREAM(sizeof(iter_y));
      ROS_ERROR_STREAM(cloud.point_step * cloud.width * cloud.height);
    img_pub_.publish(out_msg);
    img = cv::Mat::zeros(16,360, CV_8UC1);
    ROS_WARN("Theoretically published");
  }

  void PointcloudXYZIRSafe::setup(const velodyne_msgs::VelodyneScan::ConstPtr& scan_msg){
    DataContainerBase::setup(scan_msg);
    ROS_WARN_STREAM("!!!"<<cloud.point_step);
    ROS_WARN_STREAM(cloud.width);
    ROS_WARN_STREAM(cloud.height);

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
  {
    iter_x = iter_x + config_.init_width;
    iter_y = iter_y + config_.init_width;
    iter_z = iter_z + config_.init_width;
    iter_ring = iter_ring + config_.init_width;
    iter_intensity = iter_intensity + config_.init_width;
    iter_time = iter_time + config_.init_width;
    ++cloud.height;
  }

  void PointcloudXYZIRSafe::addPixel(uint16_t azimuth, float distance, float ring){
    //ROS_INFO_STREAM(azimuth << " RING "<< ring);
    int column = int(azimuth);
    //ROS_INFO_STREAM(column<< ",,,,, " <<img.cols);
    
    int row = int(ring);
    //std::cout << "ROW " << mismatched types ‘std::initializer_list<_Tp>’ and ‘float’row << " column " << column <<  " Index " << row+column*img.rows << "value " << distance << std::endl;
    //For testing limit up to 1 m
    float maximum = 1.0;
    
    int val = int(255*distance/config_.max_range);//std::min(float(255*distance),float(255*maximum));
    ROS_WARN_STREAM(val);

    if (column > 359){
      ROS_INFO_STREAM("SKIP "<< column);
      return;
    }

    // std::min(255,val);
    //std::cout << "ROW " << row << " column " << column  << std::endl;
    //std::cout << "IMG R " << img.rows << " IMG C "<< img.cols << " MAX " << img.rows* img.cols << std::endl;
    //img.at<int>(column+row*img.rows) = val ;
    img.at<int>(row,column) = 255;

    
    //for (int i=1; i< 10; i++)
    //    img.at<int>(row*i+img.rows) = 255 ;
    //ROS_ERROR("DONE");

    /*
     *(iter_x+ring) = x;
      *(iter_y+ring) = y;
      *(iter_z+ring) = z;
      *(iter_intensity+ring) = intensity;
      *(iter_ring+ring) = riffng;
      *(iter_time+time) = time;
    */
  }

  void PointcloudXYZIRSafe::addPoint(float x, float y, float z, uint16_t ring, const uint16_t azimuth, float distance, float intensity, float time)
  {
    auto  result = 360*azimuth/35999;//atan2 (y,x) * 180 / M_PI;
    ROS_INFO_STREAM(result <<  "  column");

    addPixel(result, distance,ring);

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


     if(config_.transform)
        transformPoint(x, y, z);

      *(iter_x+ring) = x;
      *(iter_y+ring) = y;
      *(iter_z+ring) = z;
      *(iter_intensity+ring) = intensity;
      *(iter_ring+ring) = ring;
      *(iter_time+time) = time;


      /*
    *iter_x = x;
    *iter_y = y;
    *iter_z = z;
    *iter_ring = ring;
    *iter_intensity = intensity;
    *iter_time = time;
    */
    //++cloud.width;
    //++iter_x;
    //++iter_y;
    //++iter_z;
    //++iter_ring;
    //++iter_intensity;
    //++iter_time;
  }
}

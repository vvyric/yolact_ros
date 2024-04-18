#include "pcd_processing_yolact/pcd_processing_yolact.h" // Make sure to include the correct header file



// Initialize method
bool pcd_processing::initialize(ros::NodeHandle &nh) {
    // ROS_INFO_STREAM(pcd_processing::pointcloud_topic);

    // Initialize ROS subscribers, publishers, and other members
    point_cloud_sub_ = nh.subscribe(pointcloud_topic, 1, &pcd_processing::cloudCallback, this);
    masks_sub_ = nh.subscribe("/yolact_ros/detections", 1, &pcd_processing::masksCallback, this);
    objects_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/objects_cloud", 1);
    // Initialize pointers
    raw_cloud_.reset(new cloud);
    preprocessed_cloud_.reset(new cloud);
    objects_cloud_.reset(new cloud);
    latest_Detections_msg_.reset(new yolact_ros_msgs::Detections);
    return true; // Return true if initialization is successful
}

// Update method
void pcd_processing::update(const ros::Time &time) {
    // Update the pcd_processing object

    if (is_cloud_updated) {
        // Preprocess the raw cloud
        if(!raw_cloud_preprocessing(raw_cloud_, preprocessed_cloud_)) {
            ROS_ERROR("Raw cloud preprocessing failed!");
            return;
        }

        // Cut the preprocessed cloud //TODO: pass the argument
        if(!cut_point_cloud(preprocessed_cloud_, processed_masks_, objects_cloud_)){
            ROS_ERROR("Cutting point cloud failed!");
            return;
        };

        // Publish the objects cloud
        pcl::toROSMsg(*objects_cloud_, cloudmsg_);
        ROS_INFO_STREAM("raw_cloud_:");
        ROS_INFO_STREAM(*raw_cloud_);
        ROS_INFO_STREAM("objects_cloud_:");
        ROS_INFO_STREAM(*objects_cloud_);
        objects_cloud_pub_.publish(cloudmsg_);

        // Reset the flag
        is_cloud_updated = false;
    }
}

// Raw cloud preprocessing
bool pcd_processing::raw_cloud_preprocessing(cloudPtr &input, cloudPtr &output) {
    // Implement the preprocessing logic here
    // For example, filtering, downsampling, etc.
    // Downsample the point cloud

    *output = *input;




    return true; // Return true on success
}

// Cut point cloud
bool pcd_processing::cut_point_cloud(cloudPtr &input, const std::vector<Detection> &detections, cloudPtr &objects) {
    // Implement the logic to cut the point cloud using masks
    // Point Cloud frame_id: xtion_rgb_optical_frame
    // image_raw frame_id: xtion_rgb_optical_frame
    // masks frame_id: xtion_rgb_optical_frame

    // Clear the output cloud
    *objects = *input;
    objects->points.clear();

    // Iterate over each mask
    for (const auto& detection : detections) {

        if (detection.class_name == detection.chosen_class || detection.chosen_class == "None"){
            int min_x = detection.x1;
            int min_y = detection.y1;
            int width = detection.mask_width;
            int height = detection.mask_height;

            int number_of_ones = pcd_processing::countOnes(detection.mask);
            ROS_INFO_STREAM("number_of_ones:");
            ROS_INFO_STREAM(number_of_ones);

            // Iterate over the points in the bounding box
            for (int i = min_y; i < min_y + height; ++i) {
                for (int j = min_x; j < min_x + width; ++j) {
                    // Check if the mask includes this point
                    if (detection.mask(i, j) == 1) {
                        // Calculate the index in the point cloud
                        int index = i * input->width + j;
                        if (index < input->points.size()) {
                            // Add the point to the output cloud
                            objects->points.push_back(input->points[index]);
                        }
                    }
                }
            }
        }
    }
    objects->width = objects->points.size();
    objects->height = 1;  // Setting height to 1 implies the cloud is unorganized
    objects->is_dense = false;  // Set to false if there might be NaN or invalid points


    return true;
}


// Cloud callback
void pcd_processing::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    // Handle new point cloud messages

    is_cloud_updated = true;

    pcl::fromROSMsg(*msg, *raw_cloud_);

}

// Masks callback
void pcd_processing::masksCallback(const yolact_ros_msgs::Detections::Ptr &msg) {
    // process new recieved masks
    processed_masks_ = Detections_msg_processing(msg);

}


std::vector<pcd_processing::Detection> pcd_processing::Detections_msg_processing(const yolact_ros_msgs::Detections::Ptr &Detections) {
    ROS_INFO("mask_msg_preprocessing is triggered.");

    std::vector<Detection> detections;
    
    for (const auto& Detection_msg : Detections->detections) {
        Detection detection;

        
        detection.mask = Eigen::Map<const Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        Detection_msg.mask.mask.data(), 
        Detection_msg.mask.image_shape[0], 
        Detection_msg.mask.image_shape[1]);
        
        detection.class_name = Detection_msg.class_name;
        detection.score = Detection_msg.score;
        detection.x1 = Detection_msg.box.x1;
        detection.y1 = Detection_msg.box.y1;
        detection.x2 = Detection_msg.box.x2;
        detection.y2 = Detection_msg.box.y2;
        detection.mask_width = Detection_msg.mask.width;
        detection.mask_height = Detection_msg.mask.height;
        detection.image_height = Detection_msg.mask.image_shape[0];
        detection.image_width = Detection_msg.mask.image_shape[1];
        detection.area = Detection_msg.mask.width * Detection_msg.mask.height;
        detection.chosen_class = Detection_msg.mask.chosen_class;
        detections.push_back(detection);


    }



    // Sort the masks by area
    auto compareArea = [](const Detection& a, const Detection& b) {
        return a.area < b.area;
    };
    std::sort(detections.begin(), detections.end(), compareArea);
    // Erase the masks with the largest 5 area (the background mask)
    if(detections.size() > 5) {
        detections.erase(detections.end() - 5, detections.end());
    }

    return detections;

}

int pcd_processing::countOnes(const Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> &matrix) {
    int count = 0;
    for (int i = 0; i < matrix.rows(); i++) {
        for (int j = 0; j < matrix.cols(); j++) {
            if (matrix(i, j) == 1) {
                count++;
            }
        }
    }
    return count;
}


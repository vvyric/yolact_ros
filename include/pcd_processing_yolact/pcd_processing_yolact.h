#ifndef PCD_PROCESSING_YOLACT_CLASS_H
#define PCD_PROCESSING_YOLACT_CLASS_H

#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Char.h>
#include <algorithm>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// #include <masks_msgs/maskID.h>
#include <yolact_ros_msgs/Detections.h>
#include <yolact_ros_msgs/Detection.h>
#include <yolact_ros_msgs/Box.h>
#include <yolact_ros_msgs/Mask.h>
// PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/common/common.h>

/**
* @brief: Class pcd_processing: cut RGB-D point cloud using 2D-masks generated
* by Segment Anything from Meta. 
* 
*
*/
class pcd_processing
{
private:
    const std::string pointcloud_topic;
    const std::string base_frame;
    bool is_cloud_updated;                      //!< new pointcloud recieved
    

public:
    
    // Alias:
    typedef pcl::PointXYZRGB point;             // Point Type (vector type)
    typedef pcl::PointCloud<pcl::PointXYZRGB> cloud;       // PointCloud Type (cloud vector type)
    typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr; // Cloud Pointer Type

    // Constructor and Destructor
    pcd_processing(const std::string &topic = "/xtion/depth_registered/points",
                   const std::string &frame ="base_link"):
                   pointcloud_topic(topic),base_frame(frame),is_cloud_updated(false) {

                   } // Initialize and refer to topic and frame with default values. Initialize member variables, allocate resources, etc. 
    
    ~pcd_processing(){
        // Empty destructor body
    } // Destructor

    /**
     * @brief initialize ros all subscribers/publishers, member variables
     * 
     * @param nh NodeHandle
     * @return true success
     * @return false failure
     */
    bool initialize(ros::NodeHandle &nh);

    /**
     * @brief called periodically, update the pcd_processing object
     * 
     * @param time 
     */
    void update(const ros::Time &time);

private:
    struct Detection {
        Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> mask;
        std::string class_name;
        double score;
        int x1, y1, x2, y2;  // Bounding box coordinates
        // std::vector<uint8_t> mask;  // Mask data
        int mask_width, mask_height;  // Dimensions of the mask
        int image_width, image_height;  // Dimensions of the image
        int area;
    };

    /**
     * @brief preprocessing the incoming raw point cloud, subsample and filter it.
     * 
     * @param input raw point cloud data
     * @param output filterd and subsampled cloud data
     * @return true success
     * @return false failure
     */
    bool raw_cloud_preprocessing(cloudPtr &input, cloudPtr &output);

    /**
     * @brief cut the point cloud using masks generating from SAM
     * 
     * @param input preprocessed point cloud
     * @param masks masks from SAM
     * @param objects objects that cut from point cloud
     * @return true success
     * @return false failure
     */
    bool cut_point_cloud(cloudPtr &input, const std::vector<Detection> &masks, cloudPtr &objects);

    /**
     * @brief callback function for new pointcloud subscriber
     * 
     * @param msg 
     */
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
    
    /**
     * @brief callback function for new masks subscriber
     * 
     * @param msg 
     */
    void masksCallback(const yolact_ros_msgs::Detections::Ptr &msg);

//     std::ostream& operator<<(std::ostream& os, const pcd_processing::singlemask& mask) {
//     os << "Mask ID: " << mask.maskid << "\n"
//        << "Segmentation: " << mask.segmentation << "\n"
//        << "Area: " << mask.area << "\n"
//        << "Bbox: " << mask.bbox << "\n"
//        << "Predicted IOU: " << mask.predicted_iou << "\n"
//        << "Point_coords: " << mask.point_coords << "\n"
//        << "Stability Score: " << mask.stability_score << "\n";
//        << "Crop box: " << mask.crop_box << "\n"
//     // Add other fields as needed
//     return os;
// }
    int countOnes(const Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> &matrix); 



    // Private variables

    ros::Subscriber point_cloud_sub_;           //!< Subscriber to the PointCloud data
    ros::Publisher objects_cloud_pub_;          //!< Publish objects point cloud
    ros::Subscriber masks_sub_;                 //!< Subscriber to the masks data 
    cloudPtr raw_cloud_ ;                       //!< Internal raw point cloud
    cloudPtr preprocessed_cloud_;               //!< Internal preprocessed cloud     
    cloudPtr objects_cloud_;                    //!< Internal objects point cloud
    yolact_ros_msgs::Detections::Ptr latest_Detections_msg_; //!< Internal latest maskID message
    sensor_msgs::PointCloud2 cloudmsg_; //!< save msg to cloudmsg_
    

    std::vector<Detection> processed_masks_; //!< Internal processed masks

    std::vector<Detection> Detections_msg_processing(const yolact_ros_msgs::Detections::Ptr &Detections);

    // Transformation
    tf::TransformListener tf_listener_;          //!< Access ros tf tree to get frame transformations
};


#endif
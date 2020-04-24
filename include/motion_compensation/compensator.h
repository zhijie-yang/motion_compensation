//
// Created by yang on 2020/4/24.
//

#ifndef SRC_COMPENSATOR_H
#define SRC_COMPENSATOR_H


#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/common/geometry.h>
#include <stamped_scan_msgs/Scan.h>

#define PI 3.14159265
#define SCAN_RATE 10
#define TF_RATE 10

class Compensator
{
public:
    /** \brief transform wrapper for points
     *	\param the point to be transformed, should be a point type defined in pcl
     *	\param the transform to be applied
     */
    static pcl::PointXYZI  applyTransform(pcl::PointXYZI p, tf::Transform tf);



    /** \brief compensation function for online processing
     *	\param the point cloud to be compensated
     *	\param the vector containing four transfrom closest to the stamp of the point cloud
     */
    static void onlineCompensate(const stamped_scan_msgs::Scan& _cloud, const std::vector<tf::StampedTransform>& _tf_vec);

    static void offlineCompensate(sensor_msgs::PointCloud2 cloud, std::vector<geometry_msgs::TransformStamped> tf_vec);

};


/*
 * ========================================================
 * Helper functions declared below
 */
tf::Transform tf_strip_stamp (const tf::StampedTransform & t);
double calc_azimuth (pcl::PointXYZI p);
double calc_pitch (pcl::PointXYZI p);
void print_tf (tf::Transform t);
void tf_add_to_map (const tf::StampedTransform & t);
std::vector<tf::StampedTransform> query_4_tf (const tf2_ros::Buffer &buf, const ros::Time &time);
/*
 * ========================================================
 */





#endif //SRC_COMPENSATOR_H

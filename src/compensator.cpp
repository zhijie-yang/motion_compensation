//
// Created by yang on 2020/1/28.
//

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/common/geometry.h>
#include <pcl/point_types.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <map>
#include <motion_compensation/compensator.h>
#include <motion_compensation/interpolator.h>
#include <stamped_scan_msgs/Scan.h>
#include <pcl/common/transforms.h>

std::vector<geometry_msgs::TransformStamped> tfVec;

std::vector<ros::Time> vec_tf_time;
std::map<ros::Time, tf::StampedTransform > tf_map;


ros::Publisher compensated_cloud_publisher;

void Compensator::onlineCompensate(const stamped_scan_msgs::Scan& _cloud, const std::vector<tf::StampedTransform>& _tf_vec)
{
    TransformExpr tf_expr = Interpolator::fitTrajectory(_tf_vec);
    tf::StampedTransform local_world;
    local_world = Interpolator::interpolate(tf_expr, _cloud.header.stamp);
    pcl::PointCloud<pcl::PointXYZI> ret;
    std::cerr << "Iterating through all points" << std::endl;
    unsigned long zero_count = 0;
    for (auto point : _cloud.points)
    {
        tf::StampedTransform tf = Interpolator::interpolate(tf_expr, point.time_stamp);
        if (tf.getOrigin().getX() == 0 && tf.getOrigin().getY() == 0 && tf.getOrigin().getZ() == 0)
        {
            zero_count += 1;
            continue;
        }
        tf.setRotation(tf::createIdentityQuaternion());
        pcl::PointXYZI p;
        p.x = point.position.x; p.y = point.position.y; p.z = point.position.z; p.intensity = point.intensity;
//        std::cerr << "Dist before tf: " << sqrt(p.x * p.x + p.y * p.y + p.z * p.z) << std::endl;
        tf::Transform _tf = local_world.inverse() * tf_strip_stamp(tf);
//        std::cerr << "Time of local world: " << _cloud.header.stamp << std::endl;
//        std::cerr << "Time: " << point.time_stamp << std::endl;
//        print_tf(_tf);
//        pcl::PointXYZI _p = applyTransform(applyTransform (p, tf_strip_stamp(tf)), local_world.inverse());
        pcl::PointXYZI _p = applyTransform(p, _tf);
        ret.push_back(_p);
//        std::cerr << "Dist after tf : " << sqrt(_p.x * _p.x + _p.y * _p.y + _p.z * _p.z) << std::endl;
    }
    std::cerr << "Num of zero translations: " << zero_count << std::endl;
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(ret, cloud_msg);
    cloud_msg.header = _cloud.header;
    compensated_cloud_publisher.publish(cloud_msg);
}


void Compensator::offlineCompensate(sensor_msgs::PointCloud2 cloud, std::vector<geometry_msgs::TransformStamped> tf_vec)
{
//    for (auto point : cloud)
//    {
//        Interpolator::interpolate()
//    }
}

pcl::PointXYZI Compensator::applyTransform(pcl::PointXYZI p, tf::Transform tf)
{
    pcl::PointXYZI ret;
    Eigen::Matrix3d rot;
    tf::matrixTFToEigen(tf.getBasis(), rot);
    Eigen::Vector3d trans;
    tf::vectorTFToEigen(tf.getOrigin(), trans);
    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
    mat.block<3,3>(0,0) = rot;
    mat.block<3,1>(0,3) = trans;
    Eigen::Affine3d affine;
    affine.matrix() = mat;
    /// This cast<float>() is so fucking important!!
    /// Without this explicit cast Eigen will give a shit long error!!!
    Eigen::Affine3f a = affine.cast<float>();
    ret = pcl::transformPoint(p, a);
//    ret = pcl::transformPoint(p, affine.cast<float>());

    return ret;
}

void point_cloud_callback (const stamped_scan_msgs::Scan &msg)
{
//    pointCloudBuf.push_back(pc);
//    std::cerr << "In the callback" << std::endl;
    int num_tf = 4;
    std::vector<tf::StampedTransform> tf_vec = queryTF(tfBuffer, msg.header.stamp, num_tf);
//    std::cerr << "tf_vec length: " << tf_vec.size() << std::endl;
//    std::cerr << "Querying TF completed" << std::endl;
    if (tf_vec.size() < 2)
    {
        return;
    }

//    for (auto &t : tf_vec)
//    {
//        print_tf(t);
//    }

    std::cerr << "Compensating... tf_vec length: " << tf_vec.size() << std::endl;
//    if (tf_vec.size() < num_tf)
//    {
//        return;
//    }
    Compensator::onlineCompensate(msg, tf_vec);
}


int main (int argc, char **argv)
{
    ros::init(argc, argv, "Motion_Compensation");
    ros::NodeHandle n;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Subscriber sub = n.subscribe("/velodyne_points_stamped", 1000, point_cloud_callback);

    compensated_cloud_publisher = n.advertise<sensor_msgs::PointCloud2>("/undistorted_points", 1);

    ros::spin();

    return 0;
}


/*=============================================================
 * Helper functions defined below
 */


tf::Transform tf_strip_stamp (const tf::StampedTransform & t)
{
    tf::Transform ret;
    ret.setBasis(t.getBasis());
    ret.setOrigin(t.getOrigin());
    return ret;
}


double calc_azimuth (pcl::PointXYZI p)
{
    return atan2(p.y, p.x) * 180 / PI;
}

double calc_pitch (pcl::PointXYZI p)
{
    return atan2(p.z, sqrt(p.x * p.x + p.y * p.y)) * 180 / PI;
}

void print_tf (tf::Transform t)
{
    if (t.getOrigin().getX() == 0 && t.getOrigin().getY() == 0 && t.getOrigin().getZ() == 0)
    {
        std::cerr << "Zero translation, passing.." << std::endl;
        return;
    }
    std::cerr << "transform: " << std::endl
//              << "stamp:" << t.stamp_.sec << "." << std::setw(2) << std::setfill('0') << t.stamp_.nsec << std::endl
              << "x: " << t.getOrigin().getX() << std::endl
              << "y: " << t.getOrigin().getY() << std::endl
              << "z: " << t.getOrigin().getZ() << std::endl
              << "q: " << t.getRotation().getX() << " " << t.getRotation().getY() << " "
              << t.getRotation().getZ() << " " << t.getRotation().getW()<< std::endl;
}

void tf_add_to_map (const tf::StampedTransform & t)
{
    ros::Time time = t.stamp_;
    tf_map.insert (std::pair<ros::Time, tf::StampedTransform >(time, t));
    vec_tf_time.push_back(time);
}


std::vector<tf::StampedTransform> queryTF (const tf2_ros::Buffer &buf, const ros::Time &time, const int& num_tf)
{
//    std::cerr << "Querying TF" << std::endl;
    std::vector<geometry_msgs::TransformStamped> _ret;
    geometry_msgs::TransformStamped tf_nearest;
    try
    {
        tf_nearest = buf.lookupTransform("world", "base", time);
    }catch (tf::TransformException& ex)
    {

    }
    int count = 0;
    double devi = 1.0 / TF_RATE;

    /// Tries to lookup for the transform in the future until unavailable
    std::vector<geometry_msgs::TransformStamped> future_part;
    for (int i = 1; i < num_tf / 2; i += 1)
    {
        ros::Duration _devi(i * devi);
        ros::Time _t = time + _devi;
        try
        {
            geometry_msgs::TransformStamped stamped_tf = buf.lookupTransform("world", "base", _t);
            future_part.push_back(stamped_tf);
            count += 1;
        }catch (tf::TransformException& ex)
        {
            break;
        }
    }

    std::cout << "Entering while loop in queryTF" << std::endl;
    /// Then tries to fetch all the past transform until satisfying ``num_tf''
    while (count < num_tf - 1)
    {
        ros::Duration _devi((num_tf - count) * devi);
        ros::Time _t = time - _devi;
        try
        {
            geometry_msgs::TransformStamped stamped_tf = buf.lookupTransform("world", "base", _t);
            _ret.push_back(stamped_tf);
            count += 1;
        }catch (tf::TransformException& ex)
        {
            count += 1;
            continue;
        }
    }

    std::cerr << "# past tf: " << _ret.size() << std::endl;
    /// Concatenates three parts of tf and cast into tf::StampedTransform type.
    _ret.push_back(tf_nearest);
    _ret.insert(_ret.end(),future_part.begin(), future_part.end());
    std::cerr << "# future tf: " << future_part.size() << std::endl;

    std::vector<tf::StampedTransform> ret;
    for (const auto& t : _ret)
    {
        tf::StampedTransform _tf;
        tf::transformStampedMsgToTF(t, _tf);
        ret.push_back(_tf);
    }
//    std::cerr << "ret length: " << _ret.size() << std::endl;
    return ret;
}
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


/** \brief compensation function for online processing, predicts the tf of points using past transforms
 *	\param the point cloud to be compensated
 *	\param the vector containing four transfrom closest to the stamp of the point cloud
 */
void Compensator::onlineCompensate(const stamped_scan_msgs::Scan& _cloud, const std::vector<tf::StampedTransform>& _tf_vec)
{
    TransformExpr tf_expr = Interpolator::fitTrajectory(_tf_vec, this->base_time);
    tf::StampedTransform local_world;
    local_world = Interpolator::interpolate(tf_expr, _cloud.header.stamp);
    pcl::PointCloud<pcl::PointXYZI> ret;
    for (auto point : _cloud.points)
    {
        tf::StampedTransform tf = Interpolator::interpolate(tf_expr, point.time_stamp);
        pcl::PointXYZI p;
        p.x = point.position.x; p.y = point.position.y; p.z = point.position.z; p.intensity = point.intensity;
        ret.push_back(applyTransform(applyTransform (p, tf_strip_stamp(tf)), local_world.inverse()));
    }
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

/** \brief transform wrapper for points
 *	\param the point to be transformed, should be a point type defined in pcl
 *	\param the transform to be applied
 */

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
    ret = pcl::transformPoint(p, affine.cast<float>());

    return ret;
}

Compensator::Compensator(ros::Time base_time)
{
    this->base_time = base_time;
}

void point_cloud_callback (const sensor_msgs::PointCloud2ConstPtr &pc)
{
//    pointCloudBuf.push_back(pc);
    std::cerr << "In the callback" << std::endl;
    pcl::PCLPointCloud2 pclPoints;
    pcl_conversions::toPCL(*pc, pclPoints);
    int ring = 10;
}


int main (int argc, char **argv)
{
    ros::init(argc, argv, "Motion_Compensation");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/velodyne_points", 1000, point_cloud_callback);


    compensated_cloud_publisher = n.advertise<sensor_msgs::PointCloud2>("/undistorted_points", 1);



    ros::spin();

/*
//    ros::Rate rate(10.0);
//    geometry_msgs::TransformStamped lastTfStamped;
//    lastTfStamped.header.stamp.sec = 0;
//    while (n.ok()){
//        geometry_msgs::TransformStamped transformStamped;
//        try
//        {
//            transformStamped = tfBuffer.lookupTransform("world", "base",
//                                                        ros::Time(0));
//            if (transformStamped.header.stamp != lastTfStamped.header.stamp)
//            {
////                std::cerr << "Last stamp: " << lastTfStamped.header.stamp << std::endl;
//                std::cerr << "Current stamp: " << transformStamped.header.stamp << std::endl;
////                std::cerr << "1" << std::endl;
//                tfVec.push_back(transformStamped);
////                std::cerr << "Newest stamp: " << transformStamped.header.stamp << std::endl;
//                long len = tfVec.size();
//                std::cerr << "length: " << len << std::endl;
//                if (len >= 2)
//                {
////                    ros::Time t1, t0;
////                    t1.sec = tfVec[len-1].header.stamp.sec; t1.nsec = tfVec[len-1].header.stamp.sec;
////                    t0.sec = tfVec[len-2].header.stamp.sec; t0.nsec = tfVec[len-2].header.stamp.sec;
////                    std::cerr << "t0: " << t0 << std::endl;
////                    std::cerr << "t1: " << t1 << std::endl;
////                    std::cerr << "n(t1-t0): " << t1.nsec - t0.nsec << std::endl;
////                    auto d = t1 - t0;
//                    std::cerr << "tf period from base to world: " << transformStamped.header.stamp - lastTfStamped.header.stamp << std::endl;
//                    tf::Stamped<tf::Transform> t1;
//                    tf::Stamped<tf::Transform> t0;
//                    tf::Transform t0_inv = t0.inverse();
//                    std::cerr << VNAME(t0_inv*t1) << std::endl;
//                    print_tf(t0_inv*t1);
//                }
//                lastTfStamped = transformStamped;
//            }
//
//        }
//        catch (tf2::TransformException &ex) {
//            ROS_WARN("%s",ex.what());
//            ros::Duration(1.0).sleep();
//            continue;
//        }
//        ros::spinOnce();
//        rate.sleep();
//    }
*/

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
    std::cerr << "transform: " << std::endl
              << "x: " << t.getOrigin().getX() << std::endl
              << "y: " << t.getOrigin().getY() << std::endl
              << "z: " << t.getOrigin().getZ() << std::endl
              << "q: " << t.getRotation().getX() << " " << t.getRotation().getY()
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
    std::vector<geometry_msgs::TransformStamped> _ret;
    geometry_msgs::TransformStamped tf_nearest = buf.lookupTransform("world", "base", time);
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

    /// Then tries to fetch all the past transform until satisfying ``num_tf''
    while (count <= num_tf)
    {
        ros::Duration _devi((num_tf - count) * devi);
        ros::Time _t = time - _devi;
        try
        {
            geometry_msgs::TransformStamped stamped_tf = buf.lookupTransform("world", "base", _t);
            _ret.push_back(stamped_tf);
        }catch (tf::TransformException& ex)
        {
            continue;
        }
        count += 1;
    }


    /// Concatenates three parts of tf and cast into tf::StampedTransform type.
    _ret.push_back(tf_nearest);
    _ret.insert(_ret.end(),future_part.begin(), future_part.end());

    std::vector<tf::StampedTransform> ret;
    for (const auto& t : _ret)
    {
        tf::StampedTransform _tf;
        tf::transformStampedMsgToTF(t, _tf);
        ret.push_back(_tf);
    }


}
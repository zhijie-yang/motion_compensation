//
// Created by yang on 2020/4/20.
//

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <vector>
#include <motion_compensation/interpolator.h>


tf::StampedTransform
Interpolator::forwardInterpolate(const TransformExpr &tf_expr, ros::Time t)
{
    /// Fits into a function w.r.t. time

    /// The order of the function is determined by the length of tf_vec
    /// order = tf_vec.size() - 1

    /// Predicts the transfrom from base to world at given time t
}

geometry_msgs::TransformStamped
Interpolator::interpolate(const TransformExpr &tf_expr, ros::Time t)
{
    /// Fits into a function w.r.t. time

    /// The order of the function is determined by the length of tf_vec
    /// order = tf_vec.size() - 1


    /// Calculate the transform from base to world at given time t
    return geometry_msgs::TransformStamped();
}

TransformExpr Interpolator::fitTrajectory(const std::vector<tf::StampedTransform>& tf_vec)
{
    TransformExpr tf_expr(3);
    return tf_expr;
}

TransformExpr::TransformExpr(int order, const std::vector<std::vector<int>> &translation_param,
                             const std::vector<std::vector<int>> &rotation_param)
{
    this->order = order;
    this->translation_param = translation_param;
    this->rotation_param = rotation_param;
}

TransformExpr::TransformExpr(int order)
{
    this->order = order;
    this->translation_param.resize(order + 1);
    this->rotation_param.resize(order + 1);
}

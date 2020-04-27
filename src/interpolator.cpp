//
// Created by yang on 2020/4/20.
//

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <vector>
#include <motion_compensation/interpolator.h>


tf::StampedTransform
Interpolator::interpolate(const TransformExpr &tf_expr, ros::Time t)
{
    /// Fits into a function w.r.t. time

    /// The order of the function is determined by the length of tf_vec
    /// order = tf_vec.size() - 1


    /// Calculate the transform from base to world at given time t

}

TransformExpr Interpolator::fitTrajectory(const std::vector<tf::StampedTransform> &tf_vec, ros::Time base_time)
{
    TransformExpr tf_expr(tf_vec.size(), base_time);
    std::vector<cv::Point> x_points;
    std::vector<cv::Point> y_points;
    std::vector<cv::Point> z_points;
    for (auto & tf : tf_vec)
    {
        long time = Interpolator::convertToExprTime(tf.stamp_);
        cv::Point2l x_point(time, (long) (tf.getOrigin().getX() * 1000));
    }

    return tf_expr;
}

Interpolator::Interpolator(ros::Time base_time)
{
    this->base_time = base_time;
}

TransformExpr::TransformExpr(int num_terms, const std::vector<std::vector<int>> &translation_param,
                             const std::vector<std::vector<int>> &rotation_param, ros::Time base_time)
{
    this->base_time = base_time;
    this->num_terms = num_terms;
    this->translation_param = translation_param;
    this->rotation_param = rotation_param;
}

TransformExpr::TransformExpr(int num_terms, ros::Time base_time)
{
    this->base_time = base_time;
    this->num_terms = num_terms;
    this->translation_param.resize(num_terms + 1);
    this->rotation_param.resize(num_terms + 1);
}


cv::Mat Interpolator::polyFit(std::vector<cv::Point> &in_point, int n)
{
    int size = in_point.size();
    //所求未知数个数
    int x_num = n + 1;
    //构造矩阵U和Y
    cv::Mat mat_u(size, x_num, CV_64F);
    cv::Mat mat_y(size, 1, CV_64F);

    for (int i = 0; i < mat_u.rows; ++i)
        for (int j = 0; j < mat_u.cols; ++j)
        {
            mat_u.at<double>(i, j) = pow(in_point[i].x, j);
        }

    for (int i = 0; i < mat_y.rows; ++i)
    {
        mat_y.at<double>(i, 0) = in_point[i].y;
    }

    //矩阵运算，获得系数矩阵K
    cv::Mat mat_k(x_num, 1, CV_64F);
    mat_k = (mat_u.t() * mat_u).inv() * mat_u.t() * mat_y;
    std::cout << mat_k << std::endl;
    return mat_k;
}

long Interpolator::convertToExprTime(ros::Time t)
{
    ///Narrow precision into microseconds
    return ((long) (t.sec * 1e6)) + ((long) (t.nsec / 1e3));
}

//
// Created by yang on 2020/4/20.
//

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <vector>
#include <motion_compensation/interpolator.h>

tf::StampedTransform
Interpolator::interpolate(const TransformExpr &tf_expr, ros::Time _t)
{
    /// Fits into a function w.r.t. time

    /// The order of the function is determined by the length of tf_vec
    /// order = tf_vec.size() - 1
    long t = Interpolator::convertToExprTime(_t);
    double x = Interpolator::calcCoord(tf_expr.translation_param[0], t);
    double y = Interpolator::calcCoord(tf_expr.translation_param[1], t);
    double z = Interpolator::calcCoord(tf_expr.translation_param[2], t);
    tf::Vector3 origin(x, y, z);
    tf::StampedTransform tf;
    tf.setOrigin(origin);


    return tf;
    /// Calculate the transform from base to world at given time t

}

TransformExpr Interpolator::fitTrajectory(const std::vector<tf::StampedTransform> &tf_vec)
{
//    TransformExpr tf_expr(tf_vec.size(), base_time);
    std::vector<cv::Point> x_points;
    std::vector<cv::Point> y_points;
    std::vector<cv::Point> z_points;
    for (auto & tf : tf_vec)
    {
        long time = Interpolator::convertToExprTime(tf.stamp_);
        cv::Point2l x_point(time, (long) (tf.getOrigin().getX() * 1e6));
        cv::Point2l y_point(time, (long) (tf.getOrigin().getY() * 1e6));
        cv::Point2l z_point(time, (long) (tf.getOrigin().getZ() * 1e6));
        x_points.push_back(x_point);
        y_points.push_back(y_point);
        z_points.push_back(z_point);
    }
    std::vector<std::vector<double>> translation_param_vec;
    translation_param_vec[0] = Interpolator::polyFit(x_points, tf_vec.size());
    translation_param_vec[1] = Interpolator::polyFit(y_points, tf_vec.size());
    translation_param_vec[2] = Interpolator::polyFit(z_points, tf_vec.size());
    std::vector<std::vector<double>> rotation_param_vec;
    TransformExpr tf_expr(tf_vec.size(), translation_param_vec, rotation_param_vec);

    return tf_expr;
}

TransformExpr::TransformExpr(int num_terms, const std::vector<std::vector<double>> &translation_param,
                             const std::vector<std::vector<double>> &rotation_param)
{
    this->num_terms = num_terms;
    this->translation_param = translation_param;
    this->rotation_param = rotation_param;
}

TransformExpr::TransformExpr(int num_terms)
{
    this->num_terms = num_terms;
    this->translation_param.resize(num_terms + 1);
    this->rotation_param.resize(num_terms + 1);
}

std::vector<double> Interpolator::polyFit(std::vector<cv::Point> &in_point, int n)
{
    int size = in_point.size();
    //所求未知数个数
    int x_num = n;
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
    std::vector<double> ret;
    for (int i = 0; i < mat_k.rows; i += 1)
    {
        ret.push_back(mat_k.at<double>(i, 0));
    }
    return ret;
}

long Interpolator::convertToExprTime(ros::Time t)
{
    ///Narrow precision into microseconds
    return ((long) (t.sec * 1e6)) + ((long) (t.nsec / 1e3));
}

double Interpolator::calcCoord(const std::vector<double>& param_vec, long expr_time)
{
    double ret = 0;
    for (int i = 0; i < param_vec.size(); i += 1)
    {
        ret += param_vec[i] * pow(expr_time, i);
    }
    return ret;
}

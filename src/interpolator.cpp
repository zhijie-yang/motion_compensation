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
    double t = tf_expr.convertToExprTime(_t);
    std::vector<double> coord;
    coord = tf_expr.calcCoord(t);
    double x = coord[0];
    double y = coord[1];
    double z = coord[2];
    tf::Vector3 origin(x, y, z);
    tf::StampedTransform tf;
    tf.stamp_ = _t;
    tf.setOrigin(origin);
    tf::Quaternion q = tf::createIdentityQuaternion();
    if (tf_expr.rotation_param.size() == 2)
    {
        q = Slerp(tf_expr.rotation_param[0], tf_expr.rotation_param[1], t);
    }

    tf.setRotation(q);

    return tf;
    /// Calculate the transform from base to world at given time t

}

TransformExpr Interpolator::fitTrajectory(const std::vector<tf::StampedTransform> &tf_vec)
{
//    TransformExpr tf_expr(tf_vec.size(), base_time);
    std::vector<cv::Point2d> x_points;
    std::vector<cv::Point2d> y_points;
    std::vector<cv::Point2d> z_points;
    TransformExpr tf_expr(tf_vec.size(), tf_vec[0].stamp_);
    for (auto & tf : tf_vec)
    {
        double time = tf_expr.convertToExprTime(tf.stamp_);
        cv::Point2d x_point(time, (tf.getOrigin().getX()));
        cv::Point2d y_point(time, (tf.getOrigin().getY()));
        cv::Point2d z_point(time, (tf.getOrigin().getZ()));
        x_points.push_back(x_point);
        y_points.push_back(y_point);
        z_points.push_back(z_point);
    }
    std::vector<std::vector<double>> translation_param_vec;
    translation_param_vec.resize(3);
    translation_param_vec[0] = Interpolator::polyFit(x_points, tf_vec.size());
    translation_param_vec[1] = Interpolator::polyFit(y_points, tf_vec.size());
    translation_param_vec[2] = Interpolator::polyFit(z_points, tf_vec.size());
    tf_expr.setTranslationParam(translation_param_vec);
    std::vector<tf::Quaternion> rotation_param_vec;
    rotation_param_vec.reserve(tf_vec.size());
    for (auto & tf : tf_vec)
    {
        rotation_param_vec.push_back(tf.getRotation());
    }
    return tf_expr;
}

tf::Quaternion TransformExpr::quaternionInterpolation(const std::vector<tf::StampedTransform> &tf_vec)
{

}

TransformExpr::TransformExpr(int num_terms, ros::Time t, const std::vector<std::vector<double>> &translation_param,
                             const std::vector<tf::Quaternion> &rotation_param)
{
    this->num_terms = num_terms;
    this->translation_param = translation_param;
    this->rotation_param = rotation_param;
    this->base_stamp = t.sec;
}

TransformExpr::TransformExpr(int num_terms, ros::Time t)
{
    this->num_terms = num_terms;
    this->translation_param.resize(num_terms + 1);
    this->rotation_param.resize(num_terms + 1);
    this->base_stamp = t.sec;
}

std::vector<double> Interpolator::polyFit(std::vector<cv::Point2d> &in_point, int n)
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
//    std::cout << mat_k << std::endl;
    std::vector<double> ret;
//    for (int i = 0; i < mat_k.rows; i += 1)
    for (int i = mat_k.rows - 1; i >= 0; i -= 1)
    {
        double _param = mat_k.at<double>(i, 0);
        if (i == 0 && _param == 0)
        {
            for (const auto& p : in_point)
            {
                std::cout << p.x << " " << p.y << std::endl;
            }
        }
        ret.push_back(_param);
    }
//    std::cout << "param_vec_length: " << ret.size() << std::endl;
    return ret;
}

double TransformExpr::convertToExprTime(ros::Time t) const
{
    ///Narrow precision into microseconds
    double _sec = t.sec - this->base_stamp;
    return _sec + (t.nsec / 1e9);
}

void TransformExpr::setTranslationParam(const std::vector<std::vector<double>> &_translation_param)
{
    this->translation_param = _translation_param;
}

void TransformExpr::setRotationParam(const std::vector<tf::Quaternion> &_rotation_param)
{
    this->rotation_param = _rotation_param;
}

std::vector<double> TransformExpr::calcCoord(double expr_time) const
{
    std::vector<double> ret;
    for (int dim = 0; dim < 3; dim += 1)
    {
        double _ret = 0;
        for (int i = 0; i < this->translation_param[dim].size(); i += 1)
//        for (unsigned long i = this->translation_param[dim].size() - 1; i >= 0; i -= 1 )
        {
            _ret += this->translation_param[dim][i] * pow(expr_time, i);
//            std::cerr << "Translation param " << dim << ": " << this->translation_param[dim][i] << std::endl;
        }
//        std::cerr << "Calculated coord " << dim << ": " << _ret << std::endl;
        ret.push_back(_ret);
    }
    return ret;
}

tf::Quaternion Slerp(const tf::Quaternion & q0, const tf::Quaternion & q1, double t)
{
    double _t = t / ( 1.0 / TF_RATE);
    if (_t > 1)
    {
        ROS_ERROR("Quaternion interpolation time exceeded upper bound, take 1.");
        _t = 1;
    }else if (_t < 0)
    {
        ROS_ERROR("Quaternion interpolation time exceeded lower bound, take 0.");
        _t = 0;
    }
    return q0.slerp(q1, _t);
}
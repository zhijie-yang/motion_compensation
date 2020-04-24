//
// Created by yang on 2020/4/24.
//

#ifndef SRC_INTERPOLATOR_H
#define SRC_INTERPOLATOR_H

class TransformExpr
{
public:
    unsigned int order;
    /// Suppose the translation in x, y and z are functions w.r.t time t in third order (fit with four stamped tf)
    /// x = x(t) = a_x*x^3 + b_x*x^2 + c_x*x + d_x
    /// The parameters to express the translation, should be given as [x_param, y_param, z_param]
    std::vector<std::vector<int>> translation_param;
    /// TODO: Decide what is the formula to express the rotation functions
    /// Suppose the rotation given in quaternion is a function w.r.t. time t in the third order
    std::vector<std::vector<int>> rotation_param;


    TransformExpr(int order, const std::vector<std::vector<int>>& translation_param, const std::vector<std::vector<int>>& rotation_param);
    explicit TransformExpr(int order);
};


class Interpolator
{
public:
    static tf::StampedTransform forwardInterpolate (const TransformExpr& tf_expr, ros::Time t);
    static geometry_msgs::TransformStamped interpolate (const TransformExpr& tf_expr, ros::Time t);
    static TransformExpr fitTrajectory (const std::vector<tf::StampedTransform>& tf_vec);


};



#endif //SRC_INTERPOLATOR_H

//
// Created by yang on 2020/4/24.
//

#ifndef MOTION_COMPENSATION_INTERPOLATOR_H
#define MOTION_COMPENSATION_INTERPOLATOR_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

class TransformExpr
{
public:
    ros::Time base_time;

    unsigned long num_terms;
    /// Suppose the translation in x, y and z are functions w.r.t time t in third order (fit with four stamped tf)
    /// x = x(t) = a_x*x^3 + b_x*x^2 + c_x*x + d_x
    /// The parameters to express the translation, should be given as [x_param, y_param, z_param]
    std::vector<std::vector<int>> translation_param;
    /// TODO: Decide what is the formula to express the rotation functions
    /// Suppose the rotation given in quaternion is a function w.r.t. time t in the third order
    std::vector<std::vector<int>> rotation_param;


    TransformExpr(int order, const std::vector<std::vector<int>>& translation_param, const std::vector<std::vector<int>>& rotation_param, ros::Time base_time);
    TransformExpr(int order, ros::Time base_time);
};


class Interpolator
{
public:
    ros::Time base_time;
    explicit Interpolator(ros::Time base_time);
    static tf::StampedTransform interpolate (const TransformExpr& tf_expr, ros::Time t);
    static TransformExpr fitTrajectory (const std::vector<tf::StampedTransform>& tf_vec, ros::Time base_time);
    static cv::Mat polyFit(std::vector<cv::Point>& in_point, int n);
    static long convertToExprTime(ros::Time t);
};


/* int main()
{
    //数据输入
    Point in[19] = { Point(50,120),Point(74,110),Point(98,100),Point(122,100),Point(144,80)
            ,Point(168,80),Point(192,70),Point(214,50),Point(236,40),Point(262,20)
            ,Point(282,20),Point(306,30),Point(328,40),Point(356,50),Point(376,50)
            ,Point(400,50),Point(424,50),Point(446,40),Point(468,30) };

    vector<Point> in_point(begin(in),end(in));

    //n:多项式阶次
    int n = 9;
    Mat mat_k = polyFit(in_point, n);


    //计算结果可视化
    Mat out(150, 500, CV_8UC3,Scalar::all(0));

    //画出拟合曲线
    for (int i = in[0].x; i < in[size(in)-1].x; ++i)
    {
        Point2d ipt;
        ipt.x = i;
        ipt.y = 0;
        for (int j = 0; j < n + 1; ++j)
        {
            ipt.y += mat_k.at<double>(j, 0)*pow(i,j);
        }
        circle(out, ipt, 1, Scalar(255, 255, 255), CV_FILLED, CV_AA);
    }

    //画出原始散点
    for (int i = 0; i < size(in); ++i)
    {
        Point ipt = in[i];
        circle(out, ipt, 3, Scalar(0, 0, 255), CV_FILLED, CV_AA);
    }

    imshow("9次拟合", out);
    waitKey(0);

    return 0;
} */


#endif //MOTION_COMPENSATION_INTERPOLATOR_H

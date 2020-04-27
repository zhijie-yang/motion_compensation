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
    unsigned long num_terms;
    /// Suppose the translation in x, y and z are functions w.r.t time t in third order (fit with four stamped tf)
    /// x = x(t) = a_x*x^3 + b_x*x^2 + c_x*x + d_x
    /// The parameters to express the translation, should be given as [x_param, y_param, z_param]
    std::vector<std::vector<double>> translation_param;
    /// TODO: Decide what is the formula to express the rotation functions
    /// Suppose the rotation given in quaternion is a function w.r.t. time t in the third order
    std::vector<std::vector<double>> rotation_param;


    /** \brief Constructor of class TransformExpr given number of terms and coefficients
     *	\param the number of terms in the polynomial equation
     *	\param the coefficient matrix of translation function
     *	\param the coefficient matrix of rotation function
     *	\return an instance of class TransformExpr
     */
    TransformExpr(int order, const std::vector<std::vector<double>>& translation_param, const std::vector<std::vector<double>>& rotation_param);

    /** \brief Constructor of class TransformExpr given only the number of terms
     *	\param the number of terms in the polynomial equation
     *	\return an instance of class TransformExpr
     */
    explicit TransformExpr(int order);
};


class Interpolator
{
public:
    /** \brief calculates the transform at a specific time
     *	\param the coefficients of the transform curve
     *	\param the point of time
     *	\return an instance of stamped transform
     */
    static tf::StampedTransform interpolate (const TransformExpr& tf_expr, ros::Time t);

    /** \brief calculates the coefficients of the transform curve
     *	\param the vector containing a sequence of transforms
     *	\return an instance of class TransformExpr
     */
    static TransformExpr fitTrajectory (const std::vector<tf::StampedTransform>& tf_vec);

    /** \brief The function does the fitting
     *  \param the vector containing points to be fitted into
     *	\param the number of terms in the polynomial equation
     *	\return the coefficients
     */
    static std::vector<double> polyFit(std::vector<cv::Point>& in_point, int n);

    /** \brief Converts ros time stamp into a long integer with unit at microseconds
     *  \param the ros time stamp
     *	\return microseconds with type long
     */
    static long convertToExprTime(ros::Time t);

    /** \brief Calculates the coordinate at a specific time with the expression of transform function
     *  \param the coefficients
     *	\param microseconds at the specific time
     *	\return the coordinate
     */
    static double calcCoord(const std::vector<double>& param_vec, long expr_time);
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

//
// Created by yang on 2020/4/24.
//

#ifndef MOTION_COMPENSATION_INTERPOLATOR_H
#define MOTION_COMPENSATION_INTERPOLATOR_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#define SCAN_RATE 10
#define TF_RATE 10

tf::Quaternion Slerp(const tf::Quaternion & q0, const tf::Quaternion & q1, double t);

class TransformExpr
{
public:
    unsigned long num_terms;
    double base_stamp;
    /// Suppose the translation in x, y and z are functions w.r.t time t in third order (fit with four stamped tf)
    /// x = x(t) = a_x*x^3 + b_x*x^2 + c_x*x + d_x
    /// The parameters to express the translation, should be given as [x_param, y_param, z_param]
    std::vector<std::vector<double>> translation_param;
    /// TODO: Decide what is the formula to express the rotation functions
    /// Suppose the rotation given in quaternion is a function w.r.t. time t in the third order
    std::vector<tf::Quaternion> rotation_param;


    /** \brief Constructor of class TransformExpr given number of terms and coefficients
     *	\param the number of terms in the polynomial equation
     *	\param the coefficient matrix of translation function
     *	\param the coefficient matrix of rotation function
     *	\return an instance of class TransformExpr
     */
    TransformExpr(int num_terms, ros::Time t, const std::vector<std::vector<double>>& translation_param, const std::vector<tf::Quaternion>& rotation_param);

    /** \brief Constructor of class TransformExpr given only the number of terms
     *	\param the number of terms in the polynomial equation
     *	\return an instance of class TransformExpr
     */
    TransformExpr(int num_terms, ros::Time t);

    /** \brief Converts ros time stamp into a double with accuracy to microseconds
     *  \param the ros time stamp
     *	\return time stamp with type double
     */
    double convertToExprTime(ros::Time t) const;


    void setTranslationParam(const std::vector<std::vector<double>>& _translation_param);

    void setRotationParam(const std::vector<tf::Quaternion>& _rotation_param);


    /** \brief Calculates the coordinate at a specific time with the expression of transform function
     *  \param the coefficients
     *	\param microseconds at the specific time
     *	\return the coordinate
     */
    std::vector<double> calcCoord(double expr_time) const;

    tf::Quaternion quaternionInterpolation(const std::vector<tf::StampedTransform> &tf_vec);

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
    static std::vector<double> polyFit(std::vector<cv::Point2d>& in_point, int n);

};

#endif //MOTION_COMPENSATION_INTERPOLATOR_H

/*
    Interpolator.h

    Simple class to interpolate data using splines.
    In particular, a 3rd order spline is fitted to the provided time and data vectors.

    by: Sergio Esteban (sesteban@caltech.edu)
*/

#pragma once

// standard C++
#include <iostream>
#include <sstream>

// standard Eigen library
#include <Eigen/Core>
#include <unsupported/Eigen/Splines>

const int dim = 1;      // dimension of the spline
const int order = 3;    // degree of polynomial
typedef Eigen::Spline<double, dim, order> Curve;
typedef Eigen::SplineFitting<Curve> CurveFit;

// Interpolator Class
class Interpolator {

    public:        
        
        // Constructor Destructor
        Interpolator(bool saturate_, bool saturate_zero_, bool warnings_) : saturate(saturate_), saturate_zero(saturate_zero_), warnings(warnings_) {};
        ~Interpolator() {};

        // containers for the data
        Eigen::VectorXd t;                   // time vector (unscaled)
        std::vector<Eigen::VectorXd> data;   // std vector if data eigen vectors

        // class options
        const bool saturate;      // saturate spline to nearest val  (hold)
        const bool saturate_zero; // saturate spline to zero         (zero)
        const bool warnings;      // print warnings

        // function that creates the spline
        void update_curve(Eigen::VectorXd t_, std::vector<Eigen::VectorXd> data_);

        // methods to query the spline
        Eigen::VectorXd get_value(double t);
        Eigen::VectorXd get_derivative(double t);
        Eigen::VectorXd get_second_derivative(double t);

    private:

        // order of the curve
        const int curve_order = order;

        // dimension of the data
        int data_dim;

        // max, min time and derivative scaling. 
        double t_min;
        double t_max;
        double scale;

        // container for the fitted curve, vector of curves for each dimension of the data
        std::vector<Curve> curve;

        // methods to create the spline
        void set_data(Eigen::VectorXd t_, std::vector<Eigen::VectorXd> data_);  // set the data (time and data) vectors
        Eigen::VectorXd rescale_time();                                         // rescale time to [0,1]
        Curve fit_curve(Eigen::VectorXd t, Eigen::VectorXd data);               // fit a curve
        double check_time(double time);                                         // check if time is within the range

};  
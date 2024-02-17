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
        
        // Constructor Desctructor
        Interpolator() {};
        ~Interpolator() {};

        // containers for the data
        Eigen::VectorXd t;                 // time vector (unscacled)
        std::vector<Eigen::VectorXd> data; // std vector if data eigen vectors

        // function that creates the spline
        void update_curve(Eigen::VectorXd t_, std::vector<Eigen::VectorXd> data_);

        // methods to query the spline
        Eigen::VectorXd get_value(double t);
        Eigen::VectorXd get_derivative(double t);
        Eigen::VectorXd get_second_derivative(double t);

    private:

        // dimension of the data
        int data_dim;

        // max and minimum time
        double t_min;
        double t_max;

        // container for the fitted curve, vecotr of curves for each dimension of the data
        std::vector<Curve> curve;

        // methods to create the spline
        void set_data(Eigen::VectorXd t_, std::vector<Eigen::VectorXd> data_); // set the data (time and data vectors
        Eigen::VectorXd rescale_time(); // rescale time to [0,1]
        Curve fit_curve(Eigen::VectorXd t, Eigen::VectorXd data); // fit a curve

};  
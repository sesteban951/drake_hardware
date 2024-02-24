/*
    Liner Interpolator class:
    This class is used to create a linear interpolation of a given data set.

    Sergio Esteban (sesteban@caltech.edu)
*/

#pragma once

// standard C++
#include <iostream>
#include <sstream>
#include <vector>

// standard Eigne library
#include <Eigen/Core>

typedef std::vector<double> TimeList;
typedef std::vector<Eigen::VectorXd> DataList;

class LinearInterpolator {

    public:

        // Constructor Desctructor
        LinearInterpolator(bool hold_, bool warnings_) : hold(hold_), warnings(warnings_) {};
        ~LinearInterpolator() {};

        // containers for the data
        TimeList t;             // time vector
        DataList data; // std vector of each data point

        // class options
        const bool hold;     // hold last value or go to zero
        const bool warnings; // print warnings

        // method to create the curve
        void update_curve(TimeList t_, DataList data_);

        // method to query the curve
        Eigen::VectorXd get_value(double time);

    private:

        // dimension of the curve
        int data_dim;

        // length of data points
        int N;

        // min and max time
        double t_min;
        double t_max;

        // methods to create the spline
        void set_data(TimeList t_, DataList data_); 

};
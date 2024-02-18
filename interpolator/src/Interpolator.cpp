/*
    Interpolator.h

    Simple class to interpolate data using splines.
    In particular, a 3rd order spline is fitted to the provided time and data vectors.

    by: Sergio Esteban (sesteban@caltech.edu)
*/

// header file for the library
#include "../inc/Interpolator.h"

// namespaces
using namespace Eigen;
using namespace std;

// set the class time and data array
void Interpolator::set_data(VectorXd t_, std::vector<VectorXd> data_) {
    
    // set the time and data
    t = t_;
    data = data_;
    data_dim = data_.size();

    // assert that the t vector and all data vecotrs are same length
    for (int i = 0; i < data.size(); i++) {
        std::stringstream ss;
        ss << "Size of t and data[" << i << "] do not match";
        assert((t.size() == data[i].size()) && ss.str().c_str());
    }
}

// rescale the time array from 0 to 1
VectorXd Interpolator::rescale_time() {
    
    // container for scaled time
    VectorXd t_rescaled;
    t_rescaled.resize(t.size());
    
    // minimal and maximal time, derivative scaling
    t_min = t.minCoeff();
    t_max = t.maxCoeff();
    scale = 1 / (t_max - t_min);
    
    // go through each element of the time array and rescale
    for (int k = 0; k < t.size(); k++) {
        t_rescaled(k) = (t(k) - t_min) / (t_max - t_min);
    }
    
    return t_rescaled;
}

// do a curve fit for a dimension of the data
Curve Interpolator::fit_curve(VectorXd t_rescaled, VectorXd data_var) {
    
    VectorXd knots = t_rescaled;
    Curve curve = CurveFit::Interpolate(data_var.transpose(), curve_order, knots);

    return curve;
}

// update the curve with new data
void Interpolator::update_curve(VectorXd t_, std::vector<VectorXd> data_) {
    
    // set the data for the class
    set_data(t_, data_);

    // rescale the time
    VectorXd t_rescaled = rescale_time();

    // create a curve fit for each dimension of the data
    std::vector<Curve> spline;
    for (int i = 0; i < data.size(); i++) {
        Curve curve_temp = fit_curve(t_rescaled, data[i]);
        spline.push_back(curve_temp);
    }

    // set the curve
    curve = spline;
}

// check if the time is within the range
double Interpolator::check_time(double time) {
 
    // warnings and saturating
    if (warnings || saturate) {

        // print warnings
        if (warnings) {
            if (time < t_min || time > t_max) {
                std::stringstream error_msg;
                error_msg << "[WARNING] Time (" << time << ") is not in time range. Min: " << t_min << ", Max: " << t_max  << endl;
                cout << error_msg.str().c_str();
            }
        }

        // saturation considerations
        if (saturate) {

            // allow for zero to be saturated
            if (saturate_zero) {
                time = time;
            }

            // saturate to nearest value
            else {
                if (time < t_min) {time = t_min;}
                if (time > t_max) {time = t_max;}
            }
        }
    }

    return time;
}

// Evaluate the curve at a given time
Eigen::VectorXd Interpolator::get_value(double time) {

    // check if the time is within the range    
    time = check_time(time);

    double time_ = (time - t_min) / (t_max - t_min);
    
    // container for the values
    Eigen::VectorXd value;
    value.resize(data_dim);

    // go through each dimension of the data and evaluate the curve
    for (int i = 0; i < data_dim; i++) {

        if (saturate && saturate_zero) {
            if (time_ < 0 || time_ > 1) {value(i) = 0;}
            else {value(i) = curve[i].derivatives(time_, 0)(0);}
        }
        else {
            value(i) = curve[i].derivatives(time_, 0)(0) ;
        }
    }

    return value;
}

// Evaluate the first derivative of the curve at a given time
Eigen::VectorXd Interpolator::get_derivative(double time) {
    
    // check if the time is within the range    
    time = check_time(time);

    double time_ = (time - t_min) / (t_max - t_min);

    // container for the values
    Eigen::VectorXd value;
    value.resize(data_dim);

    // go through each dimension of the data and evaluate the curve
    for (int i = 0; i < data_dim; i++) {

        if (saturate && saturate_zero) {
            if (time_ < 0 || time_ > 1) {value(i) = 0;}
            else {value(i) = curve[i].derivatives(time_, 1)(1) * scale;}
        }
        else {
            value(i) = curve[i].derivatives(time_, 1)(1) * scale;
        }
    }

    return value;
}

// Evaluate the second derivative of the curve at a given time
Eigen::VectorXd Interpolator::get_second_derivative(double time) {
    
    // check if the time is within the range    
    time = check_time(time);

    double time_ = (time - t_min) / (t_max - t_min);

    // container for the values
    Eigen::VectorXd value;
    value.resize(data_dim);

    // go through each dimension of the data and evaluate the curve
    for (int i = 0; i < data_dim; i++) {

        if (saturate && saturate_zero) {
            if (time_ < 0 || time_ > 1) {value(i) = 0;}
            else {value(i) = curve[i].derivatives(time_, 2)(2) * scale * scale;}
        }
        else {
            value(i) = curve[i].derivatives(time_, 2)(2) * scale * scale;
        }
    }

    return value;
}





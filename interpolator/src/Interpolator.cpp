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
    
    // minimal and maximal time
    t_min = t.minCoeff();
    t_max = t.maxCoeff();
    
    // go through each element of the time array and rescale
    for (int k = 0; k < t.size(); k++) {
        t_rescaled(k) = (t(k) - t_min) / (t_max - t_min);
    }
    
    return t_rescaled;
}

// do a curve fit for a dimension of the data
Curve Interpolator::fit_curve(VectorXd t_rescaled, VectorXd data_var) {
    
    auto knots = t_rescaled;
    Curve curve = CurveFit::Interpolate(data_var.transpose(), 3, knots);

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

// Evaluate the curve at a given time
Eigen::VectorXd Interpolator::get_value(double t) {
    
    // assert that the time is within the range
    assert((t >= t_min) && (t <= t_max) && "Time is not within the time range");

    double t_ = (t - t_min) / (t_max - t_min);

    // container for the values
    Eigen::VectorXd value;
    value.resize(data_dim);

    // go through each dimension of the data and evaluate the curve
    for (int i = 0; i < data_dim; i++) {
        value(i) = curve[i].derivatives(t_, 0)(0);
    }

    return value;
}

// Evaluate the first derivative of the curve at a given time
Eigen::VectorXd Interpolator::get_derivative(double t) {
    
    // assert that the time is within the range
    assert((t >= t_min) && (t <= t_max) && "Time is not within time range");

    double t_ = (t - t_min) / (t_max - t_min);

    // container for the values
    Eigen::VectorXd value;
    value.resize(data_dim);

    double data_min, data_max, scale;

    // go through each dimension of the data and evaluate the curve
    for (int i = 0; i < data_dim; i++) {
        data_min = data[i].minCoeff();
        data_max = data[i].maxCoeff();
        scale = 1 / (data_max - data_min);
        // scale = 1;
        value(i) = curve[i].derivatives(t_, 1)(1) * scale;
    }

    return value;
}

// Evaluate the second derivative of the curve at a given time
Eigen::VectorXd Interpolator::get_second_derivative(double t) {
    
    // assert that the time is within the range
    assert((t >= t_min) && (t <= t_max) && "Time is not within the time range");

    double t_ = (t - t_min) / (t_max - t_min);

    // container for the values
    Eigen::VectorXd value;
    value.resize(data_dim);

    double data_min, data_max, scale;

    // go through each dimension of the data and evaluate the curve
    for (int i = 0; i < data_dim; i++) {
        data_min = data[i].minCoeff();
        data_max = data[i].maxCoeff();
        scale = 1 / (data_max - data_min);
        // scale = 1;
        value(i) = curve[i].derivatives(t_, 2)(2) * scale;
    }

    return value;
}
/*
    Liner Interpolator class:
    This class is used to create a linear interpolation of a given data set.

    Sergio Esteban (sesteban@caltech.edu)
*/

#include "../inc/LinearInterpolator.h"

using namespace Eigen;
using namespace std;

// set the time and data array
void LinearInterpolator::set_data(TimeList t_, DataList data_) {

    // make sure that the data is not empty
    std::stringstream ss;
    if (t_.size() == 0) {
        ss << "Provided time array is empty.";
        assert(t_.size() > 0 && ss.str().c_str());
    }

    if (data_.size() == 0) {
        ss << "Provided data array is empty.";
        assert(t_.size() > 0 && ss.str().c_str());
    }

    // ensure that all data is the same size;
    int size1, size2;
    for (int i = 0; i < data_.size()- 1; i++) {
        
        size1 = data_[i].size();
        size2 = data_[i+1].size();

        ss << "Data size at index [" << i << "]" << " and ["<< i+1 <<"] do not match.";
        assert(size1 == size2 && ss.str().c_str());
    }

    // set the time and data
    t = t_;
    data = data_;
    N = t_.size();
    data_dim = data[0].size();

    // set the min and max time
    t_min = t[0];
    t_max = t[N-1];

}

// update the curve fit
void LinearInterpolator::update_curve(TimeList t_, DataList data_) {

    // clear the data
    t.clear();
    data.clear();

    // set the data for the class
    set_data(t_, data_);

}

// get interpolated value at time
Eigen::VectorXd LinearInterpolator::get_value(double time) {
    
    if (time < t_min || time > t_max) {
        
        // print a warning if the time is outside the range of the data
        if (warnings) {
            std::stringstream ss;
            ss << "Time " << time << " is outside the range of the data.";
            std::cout << ss.str() << std::endl;
        }
        
        // saturate the value to the nearest value or zero out
        if (hold) {
            if (time < t_min) {
                return data[0];
            }
            else {
                return data[N-1];
            }
        }
        else {
            return Eigen::VectorXd::Zero(data_dim);
        }
    }

    // Find the two points surrounding time
    int i = 0;
    while (i < N-1 && t[i+1] < time)
        i++;

    // If time is outside the range of the data, interpolate between the nearest data points
    if (time <= t[0])
        i = 0;
    else if (time >= t[N-1])
        i = N-2;

    // Perform linear interpolation
    double t1 = t[i], t2 = t[i+1];
    Eigen::VectorXd y1 = data[i], y2 = data[i+1];
    return y1 + ((time - t1) / (t2 - t1)) * (y2 - y1);
}
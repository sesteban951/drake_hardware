#include <iostream>
#include <fstream>

#include "../inc/Interpolator.h"
using namespace Eigen;
using namespace std;

int main() {

    int spline_dimension = 1;
    int spline_order = 3;

    Interpolator interp;
    
    VectorXd t(9), q(9), v(9);
    std::vector<Eigen::VectorXd> data;

    t << 0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0;
    q << 1,   2,  -2,   3,   4,   0,   1,  -1,   2;
    // v << 0,   .25,   1,  2.25,  4,  6.25,  9,  12.25,   16; 
    v = t.array().square();
    data.push_back(q);
    data.push_back(v);

    interp.update_curve(t, data);

    // simply evaluate the spline at a given time
    VectorXd vec;
    for (int i = 0; i < t.size(); i++) {
        double t_ = t[i];
        vec = interp.get_value(t_);
        cout << "t: " << t_ << " q: " << vec[0] << " v: " << vec[1] << endl;
    }
    cout << "-------------------\n";

    // evaluate the first derivative of the spline at a given time
    for (int i = 0; i < t.size(); i++) {
        double t_ = t[i];
        vec = interp.get_derivative(t_);
        cout << "t: " << t_ << " q': " << vec[0] << " v': " << vec[1] << endl;
    }

    cout << "-------------------\n";

    // evaluate the second derivative of the spline at a given time
    for (int i = 0; i < t.size(); i++) {
        double t_ = t[i];
        vec = interp.get_second_derivative(t_);
        cout << "t: " << t_ << " q'': " << vec[0] << " v'': " << vec[1] << endl;
    }

}
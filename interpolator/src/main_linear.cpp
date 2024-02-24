#include <iostream>
#include <fstream>

#include "../inc/LinearInterpolator.h"
using namespace Eigen;
using namespace std;

int main() {

    bool hold = true;
    bool warnings = true;
    LinearInterpolator interp(hold, warnings);

    // create time and data
    std::vector<double> t;
    std::vector<VectorXd> data;

    int N = 5;
    for (int i = 0; i < N; i++) {
        t.push_back(i -0.5);
        data.push_back(VectorXd::Random(3));
    }

    // update the curve
    interp.update_curve(t, data);

    // ---------------------------------------------

    // save the original data
    ofstream file;

    file.open("../data/t_original.txt");
    for (int i = 0; i < t.size(); i++) {
        file << t[i] << "\n";
    }
    file.close();

    // save the original data
    file.open("../data/data_original.txt");
    for (int i = 0; i < data.size(); i++) {
        file << data[i].transpose() << "\n";
    }
    file.close();

    // interpolate in between
    VectorXd t_array = VectorXd::LinSpaced(500, t[0] -0.5, t[N-1] + 0.5);
    file.open("../data/t_array.txt");
    for (int i = 0; i < t_array.size(); i++) {
        file << t_array[i] << "\n";
    }
    file.close();

    // save the interpolated data
    file.open("../data/data_array.txt");
    for (int i = 0; i < t_array.size(); i++) {
        VectorXd val = interp.get_value(t_array[i]);
        file << val.transpose() << "\n";
    }
    file.close();

    // ---------------------------------------------

    // update the curve
    t.clear();
    data.clear();
    for (int i = 0; i < N; i++) {
        t.push_back(i*0.1);
        data.push_back(VectorXd::Random(3)*.5);
    }

    // update the curve
    interp.update_curve(t, data);

    // save the original data
    file.open("../data/t_original2.txt");
    for (int i = 0; i < t.size(); i++) {
        file << t[i] << "\n";
    }
    file.close();

    // save the original data
    file.open("../data/data_original2.txt");
    for (int i = 0; i < data.size(); i++) {
        file << data[i].transpose() << "\n";
    }
    file.close();

    // interpolate in between
    t_array = VectorXd::LinSpaced(500, t[0] + 0.5, t[N-1] - 0.5);
    file.open("../data/t_array2.txt");
    for (int i = 0; i < t_array.size(); i++) {
        file << t_array[i] << "\n";
    }
    file.close();

    // save the interpolated data
    file.open("../data/data_array2.txt");
    for (int i = 0; i < t_array.size(); i++) {
        VectorXd val = interp.get_value(t_array[i]);
        file << val.transpose() << "\n";
    }
    file.close();

}
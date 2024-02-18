#include <iostream>
#include <fstream>

#include "../inc/Interpolator.h"
using namespace Eigen;
using namespace std;

int main() {

    bool saturate = true;
    bool saturate_zero = false;
    bool warnings = false;
    Interpolator interp(saturate, saturate_zero, warnings);
    
    VectorXd t(9), q(9), v(9);
    std::vector<Eigen::VectorXd> data;

    // raw data
    t << -2, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0;
    q << -1, 1, 0, 5, 3, 6, -7, 8, 9;
    v << 0, 1, 2, 3, 4, 5, 6, 7, 8;

    data.push_back(q);
    data.push_back(v);    

    interp.update_curve(t, data);
  
    // create linsapce to save data [0,4]
    VectorXd t_save = VectorXd::LinSpaced(200, t.minCoeff()-1, t.maxCoeff()+1);

    // save the data via fstream
    ofstream file;

    // save the original data
    file.open("../data/data_original.txt");
    for (int i = 0; i < t.size(); i++) {
        file << t[i] << " " << q[i] << " " << v[i] << "\n";
    }
    file.close();

    // save value
    file.open("../data/data.txt");
    for (int i = 0; i < t_save.size(); i++) {
        VectorXd val = interp.get_value(t_save[i]);
        file << t_save[i] << " " << val[0] << " " << val[1] << "\n";
    }
    file.close();

    // save first derivative
    file.open("../data/data_derivative.txt");
    for (int i = 0; i < t_save.size(); i++) {
        VectorXd val = interp.get_derivative(t_save[i]);
        file << t_save[i] << " " << val[0] << " " << val[1] << "\n";
    }
    file.close();

    // save second derivative
    file.open("../data/data_second_derivative.txt");
    for (int i = 0; i < t_save.size(); i++) {
        VectorXd val = interp.get_second_derivative(t_save[i]);
        file << t_save[i] << " " << val[0] << " " << val[1] << "\n";
    }
    file.close();

    // test the square and cube functions
    q = t.array().square();
    v = t.array().cube();
    data.clear();
    data.push_back(q);
    data.push_back(v);

    interp.update_curve(t, data);

    // savethe original data
    file.open("../data/data_original2.txt");
    for (int i = 0; i < t.size(); i++) {
        file << t[i] << " " << q[i] << " " << v[i] << "\n";
    }
    file.close();

    // save value
    file.open("../data/data2.txt");
    for (int i = 0; i < t_save.size(); i++) {
        VectorXd val = interp.get_value(t_save[i]);
        file << t_save[i] << " " << val[0] << " " << val[1] << "\n";
    }
    file.close();

    // save first derivative
    file.open("../data/data_derivative2.txt");
    for (int i = 0; i < t_save.size(); i++) {
        VectorXd val = interp.get_derivative(t_save[i]);
        file << t_save[i] << " " << val[0] << " " << val[1] << "\n";
    }
    file.close();

    // save second derivative
    file.open("../data/data_second_derivative2.txt");
    for (int i = 0; i < t_save.size(); i++) {
        VectorXd val = interp.get_second_derivative(t_save[i]);
        file << t_save[i] << " " << val[0] << " " << val[1] << "\n";
    }
    file.close();

}
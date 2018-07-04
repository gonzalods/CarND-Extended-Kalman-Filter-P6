#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>
#include <math.h>
#include "FusionEKF.h"
#include "tools.h"

using namespace std;

vector<string> split(string stringToBeSplitted, char delimeter){
    stringstream ss(stringToBeSplitted);
    string item;
	vector<string> splittedStrings;
    while (getline(ss, item, delimeter))
	{
		splittedStrings.push_back(item);
    }
	return splittedStrings;
}

int main(){
    // Create a Kalman Filter instance
    FusionEKF fusionEKF;

    // used to compute the RMSE later
    Tools tools;
    vector<VectorXd> estimations;
    vector<VectorXd> ground_truth;   
    
    string sensor_measurment;
    ifstream ifs ("../data/obj_pose-laser-radar-synthetic-input.txt");
    ofstream ofs ("../data/obj_pose-laser-radar-ekf-output.txt");
    
    if(ifs.is_open() && ofs.is_open()){
        while(getline(ifs, sensor_measurment)){
            //vector<string> splittedStr = split(sensor_measurment, '\t');
            istringstream iss(sensor_measurment);
            long long timestamp;
            MeasurementPackage meas_package;
            
            string sensor_type;
            iss >> sensor_type;
            if (sensor_type.compare("L") == 0) {
                //continue;
                meas_package.sensor_type_ = MeasurementPackage::LASER;
          		meas_package.raw_measurements_ = VectorXd(2);
          		double px;
      	  		double py;
          		iss >> px;
          		iss >> py;
          		meas_package.raw_measurements_ << px, py;
          		iss >> timestamp;
          		meas_package.timestamp_ = timestamp;
            }else if(sensor_type.compare("R") == 0){
                //continue;
                meas_package.sensor_type_ = MeasurementPackage::RADAR;
          		meas_package.raw_measurements_ = VectorXd(3);
          		double ro;
      	  		double theta;
      	  		double ro_dot;
          		iss >> ro;
          		iss >> theta;
          		iss >> ro_dot;
          		meas_package.raw_measurements_ << ro,theta, ro_dot;
          		iss >> timestamp;
          		meas_package.timestamp_ = timestamp;
            }
            
            float x_gt;
            float y_gt;
            float vx_gt;
            float vy_gt;
            iss >> x_gt;
            iss >> y_gt;
            iss >> vx_gt;
            iss >> vy_gt;
            VectorXd gt_values(4);
            gt_values(0) = x_gt;
            gt_values(1) = y_gt; 
            gt_values(2) = vx_gt;
            gt_values(3) = vy_gt;
            ground_truth.push_back(gt_values);
            
            //Call ProcessMeasurment(meas_package) for Kalman filter
            fusionEKF.ProcessMeasurement(meas_package);    	  

            //Push the current estimated x,y positon from the Kalman filter's state vector

            VectorXd estimate(4);

            double p_x = fusionEKF.ekf_.x_(0);
            double p_y = fusionEKF.ekf_.x_(1);
            double v1  = fusionEKF.ekf_.x_(2);
            double v2 = fusionEKF.ekf_.x_(3);

            estimate(0) = p_x;
            estimate(1) = p_y;
            estimate(2) = v1;
            estimate(3) = v2;

            estimations.push_back(estimate);

            VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);
            
            ostringstream oss;
            oss << std::fixed << std::showpos << std::setprecision(6);
            oss << "Estimation[" << std::setw(10) << p_x << ", " << std::setw(10) << p_y << ", " << std::setw(10) << v1 << ", " << std::setw(10) << v2 << "]\t";
            oss << "Ground Truth[" << std::setw(10) << x_gt << ", " << std::setw(10) << y_gt << ", " << std::setw(10) << vx_gt << ", " << std::setw(10) << vy_gt << "]\t";
            oss << "RMS[" << std::setw(10) << RMSE(0) << ", " << std::setw(10) << RMSE(1) << ", " << std::setw(10) << RMSE(2) << ", " << std::setw(10) << RMSE(3) << "]\n"; 
            string recout = oss.str();
            ofs << recout;
        }
        ifs.close();
        ofs.close();
    }else{
        cout << "Unable to open files" << endl;
    }
    return 0;
}


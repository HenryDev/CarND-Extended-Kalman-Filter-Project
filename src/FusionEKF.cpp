#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
    is_initialized_ = false;

    previous_timestamp_ = 0;

    // initializing matrices
    R_laser_ = MatrixXd(2, 2);
    R_radar_ = MatrixXd(3, 3);
    H_laser_ = MatrixXd(2, 4);
    Hj_ = MatrixXd(3, 4);

    //measurement covariance matrix - laser
    R_laser_ << 0.0225, 0,
            0, 0.0225;

    //measurement covariance matrix - radar
    R_radar_ << 0.09, 0, 0,
            0, 0.0009, 0,
            0, 0, 0.09;

    /**
    TODO:
      * Finish initializing the FusionEKF.
      * Set the process and measurement noises
    */
    H_laser_ << 1, 0, 0, 0,
            0, 1, 0, 0;
    ekf_.F_ = MatrixXd(4, 4);
    ekf_.F_ << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;
//    acceleration_noise_x = 9;
//    acceleration_noise_y = 9;
//    noise_ax = 5;
//    noise_ay = 5;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


    /*****************************************************************************
     *  Initialization
     ****************************************************************************/
    if (!is_initialized_) {
        /**
        TODO:
          * Initialize the state ekf_.x_ with the first measurement.
          * Create the covariance matrix.
          * Remember: you'll need to convert radar from polar to cartesian coordinates.
        */
        // first measurement
        cout << "EKF: " << endl;
        ekf_.x_ = VectorXd(4);
        ekf_.x_ << 1, 1, 1, 1;

        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            /**
            Convert radar from polar to cartesian coordinates and initialize state.
            */
            ekf_.x_(0) = measurement_pack.raw_measurements_[0] * cos(measurement_pack.raw_measurements_[1]);
            ekf_.x_(1) = measurement_pack.raw_measurements_[0] * sin(measurement_pack.raw_measurements_[1]);
        } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            /**
            Initialize state.
            */
            ekf_.x_(0) = measurement_pack.raw_measurements_.x();
            ekf_.x_(1) = measurement_pack.raw_measurements_.y();
        }
        ekf_.F_ = MatrixXd(4, 4);
        ekf_.F_ << 1, 0, 1, 0,
                0, 1, 0, 1,
                0, 0, 1, 0,
                0, 0, 0, 1;
        previous_timestamp_ = measurement_pack.timestamp_;
        // done initializing, no need to predict or update
        is_initialized_ = true;
        return;
    }

    /*****************************************************************************
     *  Prediction
     ****************************************************************************/

    /**
     TODO:
       * Update the state transition matrix F according to the new elapsed time.
        - Time is measured in seconds.
       * Update the process noise covariance matrix.
       * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
     */
    double delta_t = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = measurement_pack.timestamp_;

    double delta_t_squared = delta_t * delta_t;
    double delta_t_cubed = delta_t_squared * delta_t;
    double delta_t_4th = delta_t_cubed * delta_t;

    ekf_.Q_ = MatrixXd(4, 4);
    int acceleration_noise_x = 9;
    int acceleration_noise_y = 9;
    ekf_.Q_ << delta_t_4th / 4 * acceleration_noise_x, 0, delta_t_cubed / 2 * acceleration_noise_x, 0,
            0, delta_t_4th / 4 * acceleration_noise_y, 0, delta_t_cubed / 2 * acceleration_noise_y,
            delta_t_cubed / 2 * acceleration_noise_x, 0, delta_t_squared * acceleration_noise_x, 0,
            0, delta_t_cubed / 2 * acceleration_noise_y, 0, delta_t_squared * acceleration_noise_y;
    ekf_.Predict();

    /*****************************************************************************
     *  Update
     ****************************************************************************/

    /**
     TODO:
       * Use the sensor type to perform the update step.
       * Update the state and covariance matrices.
     */

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        // Radar updates
    } else {
        // Laser updates
    }

    // print the output
    cout << "x_ = " << ekf_.x_ << endl;
    cout << "P_ = " << ekf_.P_ << endl;
}

%{
    Copyright (C) 2018-2020 LEIDOS.

    Licensed under the Apache License, Version 2.0 (the "License"); you may not
    use this file except in compliance with the License. You may obtain a copy of
    the License at

    http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
    WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
    License for the specific language governing permissions and limitations under
    the License.
%}
% Generate fake pinpoint data for non-moving vehicle
% 10 minutes of data coming from a stationary host vehicle pinpoint
% The data will be published at 50Hz in accorance with the real driver
% 10(min) * 60(sec/min) * 50(samples/sec) = 30000 samples
% 108 columns per row
% Data matrix has size = 30000x108

% load needed packages
pkg load statistics
pkg load quaternion


% Generates a vector of random numbers within the specified range
function randVec = randInRange(num_elements, min_val,max_val)
    randVec = (max_val-min_val).*rand(num_elements,1) + min_val;
endfunction

% Calculates the rotation matrix from xyz euler angles
% Roation Matrix and Quaturnion confirmed with http://www.andre-gaschler.com/rotationconverter/
function rotM = rotMatrixFromEulerXYZ(phi, theta, psi)
  Rx = [1, 0, 0;
        0, cos(phi), -sin(phi);
        0, sin(phi), cos(phi)];
  
  Ry = [cos(theta), 0, sin(theta);
        0, 1, 0;
        -sin(theta), 0, cos(theta)];
  
  Rz = [cos(psi), -sin(psi), 0;
        sin(psi), cos(psi), 0;
        0, 0, 1];
  
  rotM = Rx * Ry * Rz;
endfunction  

% Data file columns
SAMPLE_ID_IDX = 1;
HEADING_IDX = 2;
NAV_SRV_IDX = 3;
NAV_STATUS_IDX = 4;
NAV_LAT_IDX = 5;
NAV_LON_IDX = 6;
NAV_ALT_IDX = 7;
NAV_POS_COVR_TYPE_IDX = 8;
ODOM_TWIST_ANG_X_IDX = 9;
ODOM_TWIST_ANG_Y_IDX = 10;
ODOM_TWIST_ANG_Z_IDX = 11;
ODOM_TWIST_LIN_X_IDX = 12;
ODOM_TWIST_LIN_Y_IDX = 13;
ODOM_TWIST_LIN_Z_IDX = 14;
POINT_X_IDX = 15;
POINT_Y_IDX = 16;
POINT_Z_IDX = 17;
QUAT_W_IDX = 18;
QUAT_X_IDX = 19;
QUAT_Y_IDX = 20;
QUAT_Z_IDX = 21;
VEL_ANG_X_IDX = 22;
VEL_ANG_Y_IDX = 23;
VEL_ANG_Z_IDX = 24;
VEL_LIN_X_IDX = 25;
VEL_LIN_Y_IDX = 26;
VEL_LIN_Z_IDX = 27;
COVARINCE_ELEMENT_COUNT = 36;
POS_COVARINCE_ELEMENT_COUNT = 9;
MIN_POSE_COVAR_IDX = 28;
MIN_ODOM_TWIST_COVAR_IDX = MIN_POSE_COVAR_IDX + POS_COVARINCE_ELEMENT_COUNT;
MIN_ODOM_POSE_COVAR_IDX = MIN_ODOM_TWIST_COVAR_IDX + COVARINCE_ELEMENT_COUNT;
EXPECTED_DATA_COL_COUNT = MIN_ODOM_POSE_COVAR_IDX + COVARINCE_ELEMENT_COUNT - 1;
  
% create data matrix 
num_samples = 30000;
num_columns = EXPECTED_DATA_COL_COUNT;
data = zeros(num_samples,num_columns);

% Covariance matrix data
% use something < 10e-3 or less according to thomas
filter_accuracy_pos_east_range = [0, 0.01]; 
filter_accuracy_pos_north_range = [0, 0.01];
filter_accuracy_pos_down_range = [0, 0.01];

quaternion_covariance_x_range = [0,0.01];
quaternion_covariance_y_range = [0,0.01];
quaternion_covariance_z_range = [0,0.01];

% CONSTANTS
% In the pinpoint driver these are the only values set
NAV_SAT_SERVICE_GPS = 1;
NAV_SAT_STATUS_FIX = 0;
NAV_SAT_FIX_POSE_COVR_TYPE_DIAGONAL_KNOWN = 2;

% Seed random number generator
rand("seed", 1253891789);

% Setup normal distributions for data
heading_mu = 0;
heading_sigma = 0.01;

% GPS locations between garage and CHub
lat_mu = 38.95650423811848;
lat_sigma = 0.000005; % about 0.5 m

lon_mu = -77.15029808852705;
lon_sigma = 0.000005; % about 0.5 m

alt_mu = 70.216;
alt_sigma = 0.5; % 0.5 m

odom_angular_vel_x_mu = 0;
odom_angular_vel_y_mu = 0;
odom_angular_vel_z_mu = 0;

odom_angular_vel_sigma = 0.01;

odom_linear_vel_x_mu = 0;
odom_linear_vel_y_mu = 0;
odom_linear_vel_z_mu = 0;

odom_linear_vel_sigma = 0.01;

point_x_mu = 0;
point_y_mu = 0;
point_z_mu = 0;

point_sigma = 0.01;

point_x_mu = 0;
point_y_mu = 0;
point_z_mu = 0;

point_sigma = 0.01;

quat_x_mu = 0.0;
quat_y_mu = 0.0;
quat_z_mu = heading_mu;

quat_sigma = 0.01;

angular_vel_x_mu = 0;
angular_vel_y_mu = 0;
angular_vel_z_mu = 0;

angular_vel_sigma = 0.01;

linear_vel_x_mu = 0;
linear_vel_y_mu = 0;
linear_vel_z_mu = 0;

linear_vel_sigma = 0.01;

'Starting to build data'
% Populate data
for (row = 1:num_samples)
  data(row, SAMPLE_ID_IDX) = row;
  
  data(row, HEADING_IDX) = normrnd(heading_mu, heading_sigma);
  
  data(row, NAV_SRV_IDX) = NAV_SAT_SERVICE_GPS;
  data(row, NAV_STATUS_IDX) = NAV_SAT_STATUS_FIX;
  
  data(row, NAV_LAT_IDX) = normrnd(lat_mu, lat_sigma);
  data(row, NAV_LON_IDX) = normrnd(lon_mu, lon_sigma);
  data(row, NAV_ALT_IDX) = normrnd(alt_mu, alt_sigma);
  
  data(row, NAV_POS_COVR_TYPE_IDX) = NAV_SAT_FIX_POSE_COVR_TYPE_DIAGONAL_KNOWN;
  
  data(row, ODOM_TWIST_ANG_X_IDX) = normrnd(odom_angular_vel_x_mu, odom_angular_vel_sigma);
  data(row, ODOM_TWIST_ANG_Y_IDX) = normrnd(odom_angular_vel_y_mu, odom_angular_vel_sigma);
  data(row, ODOM_TWIST_ANG_Z_IDX) = normrnd(odom_angular_vel_z_mu, odom_angular_vel_sigma);
  
  data(row, ODOM_TWIST_LIN_X_IDX) = normrnd(odom_linear_vel_x_mu, odom_linear_vel_sigma);
  data(row, ODOM_TWIST_LIN_Y_IDX) = normrnd(odom_linear_vel_y_mu, odom_linear_vel_sigma);
  data(row, ODOM_TWIST_LIN_Z_IDX) = normrnd(odom_linear_vel_z_mu, odom_linear_vel_sigma);
  
  data(row, POINT_X_IDX) = normrnd(point_x_mu, point_sigma);
  data(row, POINT_Y_IDX) = normrnd(point_y_mu, point_sigma);
  data(row, POINT_Z_IDX) = normrnd(point_z_mu, point_sigma);
  
  tempQuat = rotm2q(rotMatrixFromEulerXYZ(normrnd(quat_x_mu, quat_sigma), normrnd(quat_y_mu, quat_sigma), normrnd(quat_z_mu, quat_sigma)));
  data(row, QUAT_W_IDX) = get(tempQuat, "w");
  data(row, QUAT_X_IDX) = get(tempQuat, "x");
  data(row, QUAT_Y_IDX) = get(tempQuat, "y");
  data(row, QUAT_Z_IDX) = get(tempQuat, "z");
  
  data(row, VEL_ANG_X_IDX) = normrnd(angular_vel_x_mu, angular_vel_sigma);
  data(row, VEL_ANG_Y_IDX) = normrnd(angular_vel_y_mu, angular_vel_sigma);
  data(row, VEL_ANG_Z_IDX) = normrnd(angular_vel_z_mu, angular_vel_sigma);
  
  data(row, VEL_LIN_X_IDX) = normrnd(linear_vel_x_mu, linear_vel_sigma);
  data(row, VEL_LIN_Y_IDX) = normrnd(linear_vel_y_mu, linear_vel_sigma);
  data(row, VEL_LIN_Z_IDX) = normrnd(linear_vel_z_mu, linear_vel_sigma);
  
  % Position covariance
  filter_accuracy_pos_east = randInRange(1, filter_accuracy_pos_east_range(1),filter_accuracy_pos_east_range(2));
  filter_accuracy_pos_north = randInRange(1, filter_accuracy_pos_north_range(1),filter_accuracy_pos_north_range(2));
  filter_accuracy_pos_down = randInRange(1, filter_accuracy_pos_down_range(1),filter_accuracy_pos_down_range(2));
  
  pose_covariance_mat = [filter_accuracy_pos_east^2, 0.0, 0.0;
                       0.0, filter_accuracy_pos_north^2, 0.0;
                       0.0, 0.0, filter_accuracy_pos_north^2
                      ];
  
  data(row, MIN_POSE_COVAR_IDX:(MIN_POSE_COVAR_IDX+POS_COVARINCE_ELEMENT_COUNT-1)) = reshape(pose_covariance_mat, 1, POS_COVARINCE_ELEMENT_COUNT);

  % Odometry pose covariance
  quaternion_covariance_x = randInRange(1, quaternion_covariance_x_range(1),quaternion_covariance_x_range(2));
  quaternion_covariance_y = randInRange(1, quaternion_covariance_y_range(1),quaternion_covariance_y_range(2));
  quaternion_covariance_z = randInRange(1, quaternion_covariance_z_range(1),quaternion_covariance_z_range(2));

  odom_pose_covariance_mat =[0, 0, 0, 0, 0, 0;
                             0, 0, 0, 0, 0, 0;
                             0, 0, 0, 0, 0, 0;
                             0, 0, 0, quaternion_covariance_x, 0, 0;
                             0, 0, 0, 0, quaternion_covariance_y, 0;
                             0, 0, 0, 0, 0, quaternion_covariance_z
                            ];  
  
  data(row, MIN_ODOM_POSE_COVAR_IDX:(MIN_ODOM_POSE_COVAR_IDX+COVARINCE_ELEMENT_COUNT-1)) = reshape(odom_pose_covariance_mat, 1, COVARINCE_ELEMENT_COUNT); 
              
  % Odometry twist covariance is not set in real driver so it will be all 0s
  data(row, MIN_ODOM_TWIST_COVAR_IDX:(MIN_ODOM_TWIST_COVAR_IDX+COVARINCE_ELEMENT_COUNT-1)) = zeros(1, COVARINCE_ELEMENT_COUNT);
end

'Done building data matrix'

csvwrite ("pinpoint_stationary.csv", data, "delimiter", ",", "precision", 15);

'Data written'

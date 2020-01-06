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

% Generate fake data for the srx_controller driver
% 10 minutes of data coming from a stationary host vehicle srx_controller
% The data will be published at 1Hz
% 10(min) * 60(sec/min) * 1(samples/sec) = 600 samples
% 9 columns per row
% Data matrix has size = 600x9

% load needed packages
pkg load statistics

% Generates a vector of random numbers within the specified range
function randVec = randInRange(num_elements, min_val,max_val)
    randVec = (max_val-min_val).*rand(num_elements,1) + min_val;
endfunction

% Data file columns

EXPECTED_DATA_COL_COUNT = 9;
SAMPLE_ID_IDX = 1;
BRAKE_DECEL_IDX = 2;
ROBOT_ENABLED_IDX = 3;
TORQUE_IDX = 4;
HARDWARE_ID_IDX = 5;
DIAG_LEVEL_IDX = 6;
DIAG_MSG_IDX = 7;
DIAG_KEY_MSG_IDX = 8;
DIAG_VALUE_IDX = 9;
  
% create data matrix 
num_samples = 600;
num_columns = EXPECTED_DATA_COL_COUNT;
% Init cell array for string assignment
data = cell(num_samples,num_columns);

% CONSTANTS
# Possible levels of operations
DIAG_LEVEL_OK = '0';
DIAG_LEVEL_WARN = '1';
DIAG_LEVEL_ERROR = '2';
DIAG_LEVEL_STALE = '3';

% Seed random number generator
rand("seed", 1253891789);

% Setup normal distributions for data
% About of applied braking
breaking_mu = 0;
breaking_sigma = 0.001;

% Amount of applied torque
torque_mu = 0;
torque_sigma = 0.01;

% Number of SigFigs
precision = 15;

'Starting to build data'
% Populate data
for (row = 1:num_samples)
  data{row, SAMPLE_ID_IDX} = num2str(row, precision);
  
  data{row, BRAKE_DECEL_IDX} = num2str(normrnd(breaking_mu, breaking_sigma), precision);
  
  data{row, ROBOT_ENABLED_IDX} = 'TRUE'; % Change if mock driver is modified to have this service logic
  data{row, TORQUE_IDX} = num2str(normrnd(torque_mu, torque_sigma), precision);
  data{row, HARDWARE_ID_IDX} = 'SRXDBWController';
  
  % Notify system of failure to set PID parameters for first 10 sec of operation
  if (row <= 10)
    data{row, DIAG_LEVEL_IDX} = DIAG_LEVEL_WARN;
    data{row, DIAG_MSG_IDX} = 'PID not set at controller';
  elseif (10 < row && row < 500) % Notify system of system ok for the period 11 sec to 8.3 min
    data{row, DIAG_LEVEL_IDX} = DIAG_LEVEL_OK;
    data{row, DIAG_MSG_IDX} = 'PID set';
  else % After 8.3 min the controller appears to error out
    data{row, DIAG_LEVEL_IDX} = DIAG_LEVEL_ERROR;
    data{row, DIAG_MSG_IDX} = 'Not receiving updates from controller';
  endif  
  
  data{row, DIAG_KEY_MSG_IDX} = 'NOTE';
  data{row, DIAG_VALUE_IDX} = 'This is fake data';
end

'Done building data matrix'
 
cell2csv ('srx_controller_stationary.csv', data, ',');

'Data written'

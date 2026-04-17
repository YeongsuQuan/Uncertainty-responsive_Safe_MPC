clc;clear;
close all;

% Add subfolders and make directory for data
addpath(genpath('.'));
if not(isfolder(gen_path({'data','vehicle'})))
  mkdir(gen_path({'data','vehicle'}));
end

% Initial simulation point 
vInit = 20; 
xInit = [0;0;0;0;0;vInit;0]; 

% Read pedestrian data
load('obs_traj_pred.mat');

% Simulation time
ts = 0.1;
sim_time = 100*ts;

% Save file location
save_file = gen_path({'data','vehicle','vehicle_'});

%% Simulate safe MPFTC
fprintf('Simulating vehicle safe MPFTC\n');
vehicle_safe_mpftc(xInit,[save_file 'safe_mpftc.mat'],sim_time,w_obs_list, position_list, Index);
fprintf('------------------------------------------\n');

%% Plot results
plot_vehicle_results;

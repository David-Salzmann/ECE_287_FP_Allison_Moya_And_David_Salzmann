clc;
clear;

%Initialize network
[params, channels] = network_setup();

%find feasible SINR targets
gamma = feasibility_search(params, channels);

% joint transmit and relay beamforming optimization
[beamformers, relay_filters] = joint_transmit_relay_beamforming(params, channels, gamma);

%Evaluate performance
evaluate_performance(params, beamformers);
results_summary(params, gamma, beamformers);

%Plot SINR targets
K = params.K;
d = params.d;
gamma_flat = reshape(gamma, 1, []);

figure;
bar(gamma_flat);
title('Target SINR per Stream');
xlabel('Stream Index');
ylabel('SINR (linear scale)');
grid on;

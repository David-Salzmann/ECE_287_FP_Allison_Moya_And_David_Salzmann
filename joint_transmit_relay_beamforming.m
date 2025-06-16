function [beamformers, relay_filters] = joint_transmit_relay_beamforming(params, channels, gamma)
    max_iter = 1;
    K = params.K; d = params.d;
    [beamformers, ~] = admm_beamforming_solver(params, channels, gamma);

    for iter = 1:max_iter
        % Relay optimization
        relay_filters = relay_optimization(params, channels, beamformers);
        [beamformers, ~] = admm_beamforming_solver(params, channels, gamma);
    end
end
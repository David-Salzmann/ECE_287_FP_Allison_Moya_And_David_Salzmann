function results_summary(params, SINRs, beamformers)
    fprintf('\n===== Beamforming Simulation Summary =====\n');
    K = params.K;
    d = params.d;
    total_streams = K * d;
    
    % Flatten SINRs
    gamma = reshape(SINRs, 1, []);
    avg_sinr = mean(gamma);
    min_sinr = min(gamma);
    max_sinr = max(gamma);

    % Total transmit power
    total_tx_power = 0;
    for k = 1:K
        for l = 1:d
            total_tx_power = total_tx_power + trace(beamformers{k,l});
        end
    end

    % Report
    fprintf('Total Streams         : %d\n', total_streams);
    fprintf('Average Achieved SINR: %.2f\n', avg_sinr);
    fprintf('Min / Max SINR        : %.2f / %.2f\n', min_sinr, max_sinr);
    fprintf('Total Transmit Power  : %.2f\n', total_tx_power);
    fprintf('===========================================\n');
end

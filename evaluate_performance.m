function evaluate_performance(params, beamformers)
    K = params.K; d = params.d; M = params.M;
    total_power = 0;

    for k = 1:K
        for l = 1:d
            total_power = total_power + trace(beamformers{k,l});
        end
    end

    fprintf('Total Transmit Power: %.2f\n', total_power);
end
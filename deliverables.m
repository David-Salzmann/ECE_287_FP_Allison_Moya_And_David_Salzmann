clc; clear; close all;

% baseline parameters to scan
SNRs = [10, 20, 30];
antenna_configs = [2, 3, 4];
relays = [1, 2];

results = struct();
idx = 1;

for snr_val = SNRs
    for M = antenna_configs
        for R = relays
            params.K = 3;
            params.d = 2;
            params.M = M;
            params.N = M;
            params.L = M;
            params.R = R;
            params.SNR_T_dB = snr_val;
            params.SNR_R_dB = snr_val;
            params.noise_power = 1;
            params.pT = 10^(snr_val/10);
            params.pR = 10^(snr_val/10);

            [params, channels] = network_setup();
            gamma = feasibility_search(params, channels);

            try
                [beamformers, relay_filters] = joint_transmit_relay_beamforming(params, channels, gamma);
                [~, SINRs] = admm_beamforming_solver(params, channels, gamma);

                avg_sinr = mean(SINRs(:));
                min_sinr = min(SINRs(:));

                results(idx).SNR = snr_val;
                results(idx).M = M;
                results(idx).R = R;
                results(idx).avg_sinr = avg_sinr;
                results(idx).min_sinr = min_sinr;
                results(idx).SINRs = SINRs;
                idx = idx + 1;
            catch err
                warning('Simulation failed at SNR=%d, M=%d, R=%d', snr_val, M, R);
            end
        end
    end
end

% --- Plotting Results ---
avg_sinrs = arrayfun(@(r) r.avg_sinr, results);
min_sinrs = arrayfun(@(r) r.min_sinr, results);

figure;
plot(avg_sinrs, 'o-', 'DisplayName', 'Average SINR');
hold on;
plot(min_sinrs, 'x-', 'DisplayName', 'Minimum SINR');
title('SINR Performance Across Configurations');
ylabel('SINR (linear scale)');
xlabel('Test Case Index');
legend;
grid on;

save('demo_results.mat', 'results');

%plotting
xticks(1:length(results));
xticklabels([]);
xlim([0.5, length(results)+0.5]);

ax = gca;
y_min = ax.YLim(1);

for i = 1:length(results)
    label = sprintf('SNR=%d\nM=%d\nR=%d', results(i).SNR, results(i).M, results(i).R);
    text(i, y_min - 0.05, label, ...
        'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'top', ...
        'FontSize', 8, ...
        'Interpreter', 'none');
end

ylim([y_min - 0.1, ax.YLim(2)]);

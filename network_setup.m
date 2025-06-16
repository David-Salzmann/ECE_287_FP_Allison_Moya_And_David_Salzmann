function [params, channels] = network_setup()
    % Network params
    params.K = 2; % num users
    params.d = 2; % streams per user
    params.R = 2; % num relays
    params.M = 4; % tx per user
    params.N = 4; % antennas per relay
    params.L = 4; % rx per user
    params.SNR_T_dB = 20;
    params.SNR_R_dB = 20;
    params.noise_power = 1;
    
    params.pT = 10^(params.SNR_T_dB/10) * params.noise_power;
    params.pR = 10^(params.SNR_R_dB/10) * params.noise_power;
    
    for k = 1:params.K
        for i = 1:params.K
            channels.J{k,i} = (randn(params.L, params.M) + 1j * randn(params.L, params.M))/sqrt(2); % direct link
        end
    end
    for r = 1:params.R
        for i = 1:params.K
            channels.Hr{r,i} = (randn(params.N, params.M) + 1j * randn(params.N, params.M))/sqrt(2);
        end
        for k = 1:params.K
            channels.Gr{k,r} = (randn(params.L, params.N) + 1j * randn(params.L, params.N))/sqrt(2);
        end
    end
end
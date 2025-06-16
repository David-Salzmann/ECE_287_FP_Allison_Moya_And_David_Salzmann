function gamma = feasibility_search(params, channels, max_trials)
    if nargin < 3, max_trials = 1; end
    K = params.K;
    d = params.d;
    M = params.M;
    L = params.L;
    N = params.N;
    R = params.R;
    gamma = zeros(K, d);
    feasible_found = false;
    noise_power = params.noise_power;

    for trial = 1:max_trials
        % Random transmit and relay beamformers
        for k = 1:K
            for l = 1:d
                U{k,l} = randn(M,1) + 1j * randn(M,1);
                U{k,l} = sqrt(params.pT / d) * U{k,l} / norm(U{k,l});
            end
        end
        for r = 1:R
            F{r} = randn(N,N) + 1j * randn(N,N);
            F{r} = sqrt(params.pR / (R*N)) * F{r} / norm(F{r}, 'fro');
        end

        SINR_trial = zeros(K, d);
        for k = 1:K
            for l = 1:d
                uk = U{k,l};

                % Signal pwr of direct + relay paths
                H_dir = channels.J{k,k} * uk;
                relay_sum = zeros(L,1);
                for r = 1:R
                    Hrk = channels.Hr{r,k};
                    Grk = channels.Gr{k,r};
                    relay_sum = relay_sum + Grk * F{r} * Hrk * uk;
                end
                desired_signal = H_dir + relay_sum;
                P_signal = norm(desired_signal)^2;

                % Interference pwr
                interference = zeros(L,1);
                for j = 1:K
                    for m = 1:d
                        if j ~= k || m ~= l
                            ujm = U{j,m};
                            H_dj = channels.J{k,j} * ujm;
                            rel_intf = zeros(L,1);
                            for r = 1:R
                                Hrj = channels.Hr{r,j};
                                Grk = channels.Gr{k,r};
                                rel_intf = rel_intf + Grk * F{r} * Hrj * ujm;
                            end
                            interference = interference + H_dj + rel_intf;
                        end
                    end
                end

                P_interf = norm(interference)^2;
                P_noise = noise_power * L;
                SINR_trial(k,l) = P_signal / (P_interf + P_noise);
            end
        end

        % make avge SINR per user as targets
        for k = 1:K
            gamma(k,:) = 0.6 * mean(SINR_trial(k,:)) * ones(1,d); 
        end

        feasible_found = true;
        break
    end

    if ~feasible_found
        error('Could not find feasible SINR target after %d trials.', max_trials);
    end
end

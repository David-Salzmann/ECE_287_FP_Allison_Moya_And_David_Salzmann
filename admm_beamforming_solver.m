function [beamformers, SINRs] = admm_beamforming_solver(params, channels, gamma)
    K = params.K; d = params.d; M = params.M; L = params.L; R = params.R;
    pT = params.pT; pR = params.pR;
    noise_power = params.noise_power;
    max_iter = 1; tol = 1e-4;

    infeasible_mask = false(K, d);
    for k = 1:K
        for l = 1:d
            X{k,l} = eye(M);
            lambda{k,l} = 0;
        end
    end

    for iter = 1:max_iter
        for k = 1:K
            for l = 1:d
                interf_sum = zeros(M,M);
                for j = 1:K
                    for m = 1:d
                        if j ~= k || m ~= l
                            H_dj = channels.J{k,j};
                            interf_sum = interf_sum + H_dj' * H_dj;
                            for r = 1:R
                                Hrj = channels.Hr{r,j};
                                Grk = channels.Gr{k,r};
                                H_eff = Grk * Hrj;
                                interf_sum = interf_sum + H_eff' * H_eff;
                            end
                        end
                    end
                end

                H_dir = channels.J{k,k};
                H_self = H_dir;
                for r = 1:R
                    Hrk = channels.Hr{r,k};
                    Grk = channels.Gr{k,r};
                    H_self = H_self + Grk * Hrk;
                end

                cvx_begin sdp quiet
                    variable Xk(M,M) hermitian
                    minimize(trace(Xk))
                    subject to
                        Xk >= 0;
                        trace(Xk) <= pT;
                        real(trace(H_self * Xk * H_self')) >= gamma(k,l) * (real(trace(interf_sum * Xk)) + noise_power * L);
                cvx_end

                if strcmp(cvx_status, 'Solved')
                    X{k,l} = Xk;
                    infeasible_mask(k,l) = false;
                else
                    warning('CVX INFEASIBLE at stream (%d,%d): %s', k, l, cvx_status);
                    X{k,l} = 1e-6 * eye(M);
                    infeasible_mask(k,l) = true;
                end
            end
        end

        err = 0;
        for k = 1:K
            for l = 1:d
                err = err + abs(trace(X{k,l}) - pT);
            end
        end
        if err < tol, break; end
    end

    SINRs = zeros(K, d);
    for k = 1:K
        for l = 1:d
            if any(isnan(X{k,l}(:))) || any(~isfinite(X{k,l}(:)))
                SINRs(k,l) = 0;
                continue;
            end

            try
                uk = sqrtm(X{k,l}) * ones(M,1);
            catch
                warning('sqrtm failed on X{%d,%d}, using fallback vector.', k, l);
                uk = ones(M,1);
            end

            % Effective channel for user k including relays
            H_total = channels.J{k,k};
            for r = 1:R
                Hrk = channels.Hr{r,k};
                Grk = channels.Gr{k,r};
                H_total = H_total + Grk * Hrk;
            end

            vk = H_total * uk;
            vk = vk / norm(vk); % matched filter

            signal = vk' * H_total * uk;
            P_signal = abs(signal)^2;

            interf = 0;
            for j = 1:K
                for m = 1:d
                    if j ~= k || m ~= l
                        try
                            ujm = sqrtm(X{j,m}) * ones(M,1);
                        catch
                            ujm = ones(M,1);
                        end
                        H_intf = channels.J{k,j};
                        for r = 1:R
                            Hrj = channels.Hr{r,j};
                            Grk = channels.Gr{k,r};
                            H_intf = H_intf + Grk * Hrj;
                        end
                        interf = interf + abs(vk' * H_intf * ujm)^2;
                    end
                end
            end

            SINRs(k,l) = P_signal / (interf + noise_power);
        end
    end

    beamformers = X;
end
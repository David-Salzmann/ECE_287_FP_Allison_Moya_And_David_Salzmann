function relay_filters = relay_optimization(params, channels, beamformers)
    R = params.R; 
    N = params.N; 
    K = params.K;
    d = params.d;
    M = params.M;
    L = params.L;
    noise_power = params.noise_power;

    relay_filters = cell(R, 1);

    for r = 1:R
        Qr = zeros(N, N);

        % Desired signal contribution
        for k = 1:K
            for l = 1:d
                try
                    uk = sqrtm(beamformers{k,l}) * ones(M,1);
                catch
                    uk = ones(M,1);
                end
                Hrk = channels.Hr{r,k};
                Grk = channels.Gr{k,r};
                signal_path = Grk * Hrk * uk;
                Qr = Qr + signal_path * signal_path';
            end
        end

        % Interference contribution
        for k = 1:K
            for l = 1:d
                for j = 1:K
                    for m = 1:d
                        if j ~= k || m ~= l
                            try
                                uj = sqrtm(beamformers{j,m}) * ones(M,1);
                            catch
                                uj = ones(M,1);
                            end
                            Hrj = channels.Hr{r,j};
                            Grk = channels.Gr{k,r};
                            interference_path = Grk * Hrj * uj;
                            Qr = Qr + 0.5 * (interference_path * interference_path');
                        end
                    end
                end
            end
        end

        cvx_begin quiet
            variable Fr(N,N) complex
            f = vec(Fr);
            Qcvx = kron(Qr.', eye(N));
            minimize(real(f' * Qcvx * f))
            subject to
                norm(Fr, 'fro') <= sqrt(params.pR / R);
        cvx_end


        if ~strcmp(cvx_status, 'Solved')
            warning('Relay SDP not solved for relay %d. Status: %s', r, cvx_status);
        end

        relay_filters{r} = Fr;
    end
end

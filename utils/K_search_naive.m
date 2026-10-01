%%% Script searching for a stable L_d, L_c 
% using naive gridsearch


sys_ball = BouncingBallSubSystemClass();

sys_obs = BouncingBallObserver();
sys_obs.L_c = [0.1; 0.25];
sys_obs.L_d = 1*[0.1; 0.1];

sys_ball.mu = 0; % Additional velocity at each impact

sys_ball.lambda = 1;
sys_ball.f_air = 0;

sys_obs.mu = sys_ball.mu;
sys_obs.lambda = sys_ball.lambda;
sys_obs.f_air = sys_ball.f_air;



% BEWARE : only true for no air friction
F = [0, 1; 0, 0];
H = sys_ball.outputJacobian();
tau = 15.6718 - 13.6045;
x = [0; -10.0995];

% Naive search

do_break = false;

for ld_1 = -5:0.01:5
    if do_break
        break
    end
    for ld_2 = -5:0.01:5
        if do_break
            break
        end
        for lc_1 = 0:0.01:0.8
            if do_break
                break
            end
            for lc_2 = 0:0.01:0.8
                if do_break
                    break
                end
                sys_obs.L_c = [lc_1; lc_2];
                sys_obs.L_d = [ld_1; ld_2]; 
                [M_before, M_after] = sys_ball.errorSaltationMatrices(x, sys_obs.L_d);
                if all(abs(eig(M_after*expm((F - sys_obs.L_c*H)*tau))) <= 1)
                    if all(abs(eig(M_before*expm((F - sys_obs.L_c*H)*tau))) <= 1)
                        if any(abs(eig( (M_before + sys_obs.L_d*H)*expm((F - sys_obs.L_c*H)*tau))) > 1)
                            disp('Unstable flow, stable hybrid');
                            disp(sys_obs.L_c);
                            disp(sys_obs.L_d);
                            do_break = true;
                        end
                    end
                end
            end
        end
    end
end



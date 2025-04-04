clear;
Nsteps = 7; % number of time steps
Nstates = 10;   % number of states

% probabilities for motion model
pRR = 0.9;  % move right command and end up advancing
pRS = 0.1;  % move right command but end up staying
pLL = 0.8;  % move left command and end up retracting
pLS = 0.2;  % move left command but end up staying

% probabilities for sensor model
pCP = 0.9;  % correct positive
pFN = 0.1;  % false negative
pCN = 0.95;  % correct negative
pFP = 0.05;  % false positive

% locations of wall
W = [2,6,7];

% sequence of actions
U = [1,1,1,-1,0,-1,0];    % from u_0 to u_6
  
%% simulated motion
x = zeros(Nsteps,1);
x(1) = randi([4, 6]);   % initial position, random between (including) 3 and 5

% measurement model for the initial position
if ismember(x(1),W)
    if rand < pCP
        Z(1) = 1;
    else
        Z(1) = 0;
    end
else
    if rand < pCN
        Z(1) = 0;
    else
        Z(1) = 1;
    end
end

% running simulaion to generate robot motion
for k = 1:Nsteps-1
    if U(k) == 1 
        if rand < pRR
            x(k+1) = x(k) + 1;  % advance to right
        else
            x(k+1) = x(k);
        end
    elseif U(k) == -1
        if rand < pLL
            x(k+1) = x(k) - 1;  % advance to right
            if x(k+1) < 1
                x(k+1) = 1;
            end
        else
            x(k+1) = x(k);
        end
    else
        x(k+1) = x(k);
    end

    % measurement model 
    if ismember(x(k+1),W)
        if rand < pCP
            Z(k+1) = 1;
        else
            Z(k+1) = 0;
        end
    else
        if rand < pCN
            Z(k+1) = 0;
        else
            Z(k+1) = 1;
        end
    end

end

%% Bayes Filter
% initialization of a (batch) belief
bel = zeros(Nsteps+1,Nstates);
% bel(xi_0) is one of {3,4,5} with equal probability
bel(1,4:6) = 1/3;

inv_eta = 0;    % the variable for 1/eta
% initialization of a (batch) bar_belief
brbel = zeros(Nsteps,Nstates);

% the main loop
for k = 1:Nsteps
    % prediction step, i.e. computing bar_belief using the motion model
    for i = 1:Nstates
        % Initialize predicted belief
        brbel(k,i) = 0;
        if U(k) == 1  % Move right
            if i > 1
                brbel(k,i) = brbel(k,i) + pRR * bel(k,i-1);
            end
            brbel(k,i) = brbel(k,i) + pRS * bel(k,i);
        elseif U(k) == -1  % Move left
            if i < Nstates
                brbel(k,i) = brbel(k,i) + pLL * bel(k,i+1);
            end
            brbel(k,i) = brbel(k,i) + pLS * bel(k,i);
        elseif U(k) == 0  % Stay
            brbel(k,i) = bel(k,i);
        end
    end

    % correction step, i.e. computing belief using the sensor output
    for i = 1:Nstates
        if ismember(i, W)
            if Z(k) == 1
                bel(k+1,i) = pCP * brbel(k,i);  % correct positive
            else
                bel(k+1,i) = pFN * brbel(k,i);  % false negative
            end
        else
            if Z(k) == 1
                bel(k+1,i) = pFP * brbel(k,i);  % false positive
            else
                bel(k+1,i) = pCN * brbel(k,i);  % correct negative
            end
        end
        inv_eta = inv_eta + bel(k+1,i);
    end
    % normalize belief using inverse of eta
    bel(k+1,:) = bel(k+1,:)/inv_eta;
    % reset the inv_eta for the next time step
    inv_eta = 0;
end

figure;
for i = 1:Nsteps+1
    subplot(Nsteps+1,1,i);plot(bel(i,:));
end

[~, most_likely_k6] = max(bel(7,:));
[~, most_likely_k7] = max(bel(8,:));
fprintf('Most likely position at k=6 is %d\n', most_likely_k6);
fprintf('Most likely position at k=7 is %d\n', most_likely_k7);

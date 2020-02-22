%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%// World/Task Paramters \\%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global L H f_states p_e gamma H_t s_0 ep
L = 5; % x-axis
H = 6; % y-axis
f_states = [1 1; 2 1; 1 3; 2 3];
p_e = 0.01;
gamma = 0.9;
H_t = 1000; % init time horizon to an arbitrarily large number
s_0 = [2 5];
ep = 0.01; % value iteration "stop" threshold

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%// Space Initializations \\%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%{
State Space
Create state space from L, H, and forbidden positions.
The discrete state space is a N_s x 2 matrix where
the rows represent the x and y value of every possible
state.
%}
global N_s S
N_s = (L*H) - size(f_states, 1);
S = zeros(N_s, 2);
n = 1; % S row index
for i = 0:L - 1
    for j = 0:H - 1
        if ~ismember([i, j], f_states, 'rows')
            S(n, :) = [i j];
            n = n + 1;
        end
    end
end

%{
"State Validity Matrix"
To improve performance, I use a state validity matrix in which each element
of the matrix indexed by the x and y coordinates of the state is 1 if valid
and 0 if not. Used by the is valid state function
%}
global S_v
S_v = zeros(H, L);
for i = 1:N_s
    S_v((S(i, 2) + 1), (S(i, 1) + 1)) = 1;
end

%{
Action Space
The action space is a N_a x 2 matrix. Instead of position 
coordinates each row represents motion in x or y.
%}
global N_a A
N_a = 5;
A = [0 1; 0 -1; 1 0; -1 0; 0 0]; %[Up, Down, Right, Left, Stop]

%{
"Reward State" Space
Contains a H x L matrix where each space represents a state and contains
a reward value, used by the reward function. Obstacles are given -999
value for debugging purposes. (-inf causes issues for element-wise 
multiplication)
%}
global S_r
S_r = [0 0 10 0 -100; 0 0 0 0 -100; 0 0 1 0 -100; ...
       0 0 0 0 -100; 0 0 0 0 -100; 0 0 0 0 -100];
   
%{
Policy Initialization
Policies are represented by H x L matricies that output the index of the
action, indexed by the state space.
%}
pi_0 = zeros(H, L);
for i = 1:N_s
    x = S(i, 1);
    y = S(i, 2);
    pi_0(y + 1, x + 1) = 4;
end

%%%%%%%%%%%%%%%%
%%//  main  \\%%
%%%%%%%%%%%%%%%%
pool = pol_iter(pi_0);
disp_pol(pool);


%%%%%%%%%%%%%%%%%%%
%%// Functions \\%%
%%%%%%%%%%%%%%%%%%%
%{
Policy Iteration Function
When given an initial policy, returns the optimal policy using
policy iteration.
%}
function pol_opt = pol_iter(pol_0)
    global H_t
    pol_temp = pol_0;

    for h = 0:H_t
        val = V(pol_temp, h);
        
        % compute next value function and policy
        [val, pol_next] = bellman(val);
        disp_pol(pol_next);
        fprintf("\n\n");
        % if the policy is optimal, return
        if pol_temp == pol_next
            break
        else
            pol_temp = pol_next;
        end
    end
    pol_opt = pol_temp;
end

%{
Value Iteration Function
When given an initial policy, returns the optimal policy using
value iteration.
%}
function pol_opt = val_iter(pol_0)
    global L H H_t ep
    pol_temp = pol_0;
    val = zeros(H, L);
    
    for h = 0:H_t
        [val_next pol_temp] = bellman(val);
        
        if abs(norm(val, inf) - norm(val_next, inf)) < ep
            break
        else
            val = val_next;
        end
    end
    disp(val);
    pol_opt = pol_temp;
end

%{
State Transition Probability Function
The state transition function takes a state, action, and a next
state and returns the probability pr that the transition will 
occur. Error probability p_e.
%}
function pr = s_trans(s_0, a, s_1)
    global A p_e
    % initialize output to 0
    pr = 0;
    
    if ~isvalid(s_0) || ~isvalid(s_1)
        return
    end
    
    % handle with a = 'no motion' case
    if isequal(a, [0 0])
        if isequal(s_0, s_1)
            pr = 1;
            return
        else
            pr = 0;
            return
        end
    end
    
    % handle a = 'motion' for edge cases and f_states (A(1:4, :))
    if ~isequal(a, [0 0]) && isequal(s_0, s_1)
        if ~isvalid(s_0 + a)
            pr = pr + (1 - p_e);
        end
        for i = 1:4
            if ~isvalid(s_0 + A(i, :))
                pr = pr + p_e/4;
            end
        end
        return
    end
    
    % handle a = 'motion' normal cases
    if s_0 + a == s_1
        pr = 1 - p_e + p_e/4;
        return
    else
        for i = 1:size(A, 1)
            if ~isequal(A(i, :), a) && ~isequal(A(i, :), [0 0]) && isequal(s_0 + A(i, :), s_1)
                pr = p_e/4;
                return
            end
        end
    end
    
    pr = 0; % for all other cases, pr = 0
end

%{
Value function matrix generator
Returns the value over the entire state space for a given policy.
%}
function val = V(pol, Hor)
    global L H gamma N_s S S_r
    val = zeros(H, L);
    
    % calculate expected value of each state transition for all s over t
    for n = 1:N_s
        temp = zeros(H, L);
        c_state = S(n, :);
        temp_state = c_state;
        for t = 0:Hor
            for i = 1:H
                for j = 1:L
                    % retrieve action on current state from policy
                    a = pie(pol, temp_state);
                    % retrieve state transition probability for (s, a, s')
                    temp(i, j) = s_trans(temp_state, a, [j - 1, i - 1]);
                end
            end
            % calculate expected sum of rewards for this state
            val(c_state(2) + 1, c_state(1) + 1) = val(c_state(2) + 1, c_state(1) + 1) ...
                                                  + (sum(temp.*S_r, 'all'))*gamma^t;
            % move to next valid state
            if isvalid(temp_state + a)
                temp_state = temp_state + a;
            end
        end
    end
end

%{
Bellman Backup
When given a value function, performs a 1-step Bellman Backup
%}
function [val_1, pol_1] = bellman(val_0)
    %TODO: add reward factor to each Q
    global L H S N_s gamma A N_a
    
    pol_1 = zeros(H, L);
    % initialize to -inf for true max
    val_1 = zeros(H, L);
    for i = 1:N_s
        x = S(i, 1);
        y = S(i, 2);
        val_1(y + 1, x + 1) = -inf;
    end
    norm_val = gamma*val_0;
    temp_s = zeros(H, L); % state transition matrix placeholder
    temp_r = zeros(H, L); % reward matrix placeholder
    
    %loop through every state action pair
    for n = 1:N_s
        x = S(n, 1);
        y = S(n, 2);
        for m = 1:N_a
            a = A(m, :);

            % obtain state transition probabilities
            for i = 1:H
                for j = 1:L
                    % retrieve state transition probability for (s, a, s')
                    temp_s(i, j) = s_trans([x y], a, [j - 1, i - 1]);
                    % retrieve rewards
                    temp_r(i, j) = r([x y] + a);
                end
            end
            % place max and its corresponding action of each state into 
            % the next step
            Q_h = sum(temp_s.*(temp_r + norm_val), 'all');
            if val_1(y + 1, x + 1) < Q_h
                val_1(y + 1, x + 1) = Q_h;
                pol_1(y + 1, x + 1) = m;
            end
        end
    end
end

%{
Reward Function
Takes a state, returns reward at that state, uses the "Reward State" 
matrix
%}
function re = r(s)
    global S_r S
    re = 0;
    if isvalid(s)
        re = S_r(s(2) + 1, s(1) + 1);
    end
    return
end

%{
Policy Function
When given a policy and state, returns the action.
%}
function a = pie(pol, s)
    global A
    index = pol(s(2) + 1, s(1) + 1);
    a = A(index, :);
    return
end

%{
State Validity Function
When given state, returns if it is valid or not
%}
function a = isvalid(s)
    global L H S_v
    if s(1) < 0 || s(1) >= L || s(2) < 0 || s(2) >= H
        a = 0;
        return
    elseif S_v(s(2) + 1, s(1) + 1) == 1
        a = 1;
        return
    else
        a = 0;
        return
    end
end

%{
Policy display Function
Displays the world space given a policy as input. Up, Down, Left, Right,
Stop, and Forbidden State are represented by ^ v < > o and - respectively
%}
function disp_pol(pol)
    global L H

    % decode and display
    row_text = "\t";
    for i = 1:H
        for j = 1:L
            % highest row corresponds to y = H - 1
            switch pol(H - (i - 1), j)
                case 0
                    row_text = strcat(row_text, "-\t");
                case 1
                    row_text = strcat(row_text, "^\t");
                case 2
                    row_text = strcat(row_text, "v\t");
                case 3
                    row_text = strcat(row_text, ">\t");
                case 4
                    row_text = strcat(row_text, "<\t");
                case 5
                    row_text = strcat(row_text, "o\t");
            end
        end
        fprintf(row_text);
        fprintf('\n');
        row_text = "\t";
    end
end


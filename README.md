# 183DA MDP_solver
robots!(but mostly theory)

Instructions:
Not much, open the script in MATLAB and hit run. The optimal policy for the base task will be displayed in the command window at each step. Top level commands start at line 84 under the "main" section.

Problem Set Questions:
Preliminaries
0a) I(Timothy Lee) completed this alone.
0b) The MATLAB bible
    https://www.mathworks.com/support.html,
    I consulted these slides when I couldn't understand the Bellman Backup implementation
    https://courses.cs.washington.edu/courses/cse473/12sp/slides/19-rl.pdf

MDP System
1a) line 24, N_s = 26
1b) line 54, N_a = 25
1c) line 146

Planning Problem
2a) Forbidden states are initialized in like 7, line 304 contains a state validity function that uses an alternate data         structre to quickly check if a state is forbidden or not. The code block starting at line 167 handles edge cases. The pdf is shifted accordingly when the robot attempts to move into a wall (or runs into a wall due to error);
2b) line 280 returns the reward given a state. Since the rewards are hard coded, this function simply refers to a "reward state" matrix initialized in line 65.

Policy Iteration
3a) line 74, all actions indexed by the state are "left" or "4" in my code since that is the row index of my "left" command in the action space matrix.
3b) line 319 the function takes a policy and decodes it to characters representing commands:
    ^ = up
    v = down
    > = right
    < = left
    0 = stop
    - = forbidden
    The output for the "all left" initial policy is:
    < < < < <
    < < < < <
    < - - < <
    < < < < <
    < - - < <
    < < < < <

3c) line 199, the function takes a policy and a time horizon input and computes the value function. The function will move the next (valid) state as prescribed by the given policy for time horizon discrete steps.
3d) line 232, the function computes a Bellman backup step on the given value function.
3e) line 92, the policy iteration function takes any initial policy and performs policy evaluation/update until the time horizon is reached or the policy converges.
3f) The total runtime determined by MATLAB was 2.696 seconds. The time horizon was set to 1000 so the policy had time to converge. The optimal policy under the given task parameters is:
    v	v	v	v	<	
    v	<	>	v	<	
    v	-	-	v	<	
    v	>	>	v	<	
    v	-	-	v	<	
    >	>	o	<	<	
3g) The reward was found by passing the optimal policy to the value function for a time horizon equal to the number of steps needed to go from initial to final (the only stop state in the policy). The expected discounted sum of rewards is, 4.7471. The lowest on the graph! This is probably due to the fact that all other states are closer and that a closer state can rack up rewards by sitting in the +10 state.

Value Iteration
4a) line 122, the value iteration function is mostly identical besides the loop conditions, that is, for an arbitrarily large time horizon, the function will only exit when the difference of the inf norms of two steps is below some small threshold, defined in line 12
4b) The policy/rewards derived are identical to the one in policy iteration part 3f).
4c) The runtime for value iteration was 16.013 seconds. This runtime was sensitive to the threshold. I initialized it at 0.01 for this time but I ran more trials to see how high I could raise the threshold without changing the output. A threshold value of 20 output the same policy in 1.982 seconds, faster than policy iteration!

Additional Scenarios (using policy iteration)
5a) If the discount factor was low (rewards were discounted very quickly over time), the policy output was:
    v v o < <
    v < < < <
    v - - v <
    > > o < <
    v - - v <
    > > o < <
    This policy seems to converge on the +1 state too since rewards die off quickly. It seems at the top where its      impossible to reach the rewards quickly, the optimal move is to give up.

    The error probability was set to 0, the policy output was:
    v v v v <
    v > > v <
    v - - v <
    v > > v <
    v - - v <
    > > o < <
    It seems when there is no chance of error, policy iteration will return the most efficient way to get the the goal(the right lane next to the dangerous street).
    
    The error probability was set to 0.5, the policy output was:
    v v v < <
    v < < < <
    v - - o <
    v < < v <
    v - - v <
    > > o < <
    This shows that if theres a good chance the robot isn't going to go where it wants to, it should take the safer, but longer way. Interestingly, the o at (3, 3) even thinks that its safer to sit there and do nothing than to accidentally run into the street.

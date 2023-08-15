

step 1

we continuously want to descibe and analyze more complex systems/tasks.

The analysis approach we take immediately depends on the way we choose to describe this complex system/task.


step 2

A promising approach for describing (say what kind of systems TL tends to be good at) systems is using temporal logic:
    because:...


step 3

why choose reachability analysis

step 3a

hybrid zonotopes vs level sets vs polytopes vs ...



step 4

Describe the process of first computing the TLT, model checking, control synthesis...
Also, this is the point where you show that in this work we are only focusing on the generation of the temporal logic tree, which can later on be
used for model checking and control synthesis.

Find a good motivation as to why we only focus on that part instead of the whole process.

step 5


******************************************************************************************************************
At each iteration, we first compute the FRS of the non-ego vehicles. Then when we have the static image of each obstacle
we compute the intersection of their complements. Then we compute the intersection of all the aforementioned intersection
with the original safe set. Doing so will allow us to now reduce the BRS component of the problem into a simple computation
of a BRS in a static environment, all while avoiding the infinite wall problem

Actually, do not do that. Cause this ignores that the non-ego vehicle is moving and so any position after the obstacle
will be removed even though it should not because by the time the ego vehicle reaches the old position of the non-ego vehicle
it might actually be safe now.

Therefore, you should instead compute the BRS on the safe space while only accounting for the static obstacles, and then
computing the intersection of that safe space, with the the safe space computed after each non-ego vehicle.



Maybe for the non-ego vehicles you do not need to consider a 4-D dynamics model. You can simply assume a 2-D dynamics model



TODO:

set the velocity in the y-direction for the down_road such that its lower bound is slightly above zero,
and make it in such a way so that the y-component will be removed smoothly alongside the x-component when transitioning from
down_road to right_road












- BRS:
    - The computation of the BRS, should be uninterrupted by the computation of the obstacles or anything else.
    - That is, compute the normal BRS ignoring the dynamic obstacles.

- Obstacles:
    - The computation of the FRS of obstacles, is not affected by anything else.
    - That is, compute the normal FRS of the moving obstacles

- Safe Space:
    - Normally compute it as the intersection of the complement of the moving obstacle and the entire state space.

- BRS Safe Space:
    - For each time step you need to compute the intersection of the complement of the obstacle at time 't' (safe space) and the
    BRS at time 't'. In addition, the result of that intersection will need to be intersected with the BRS Safe Space of the previous
    time step. 't-1'.


    - Compute the union of the BRS at time 't' with the BRS Safe Space of time 't-1'.
    - Then compute the intersection of that union with the complement of the moving obstacle at time 't'

    - Initialize BRS Safe Space as an empty space



- This procedure will result in an uninterrupted computation of the BRS and FRS, but at each time step the colliding points of the
obstacle with the BRS of that time step will be removed only from the BRS Safe Space.














# Step 1:
    - Compute the BRS ignoring all obstacles.

# Step 2:
    - Compute the RCI of all static obstacles and remove it from the BRS of step 1

# Step 3: (TODO: Think and discuss with Frank, for dynamic obstacles you do not care about RCI right? cause the car can move, so only the FRS matters)
    - Compute the FRS of all dynamic obstacles

# Step 4:
    - At each time step of the computed FRS of the dynamic obstacles do the two following things:

    # Step 4a:
        - Compute the intersection of the complement of the FRS at time 't' with the BRS of step 1 at time 't'

    # Step 4b:
        - Compute the 1-step BRS from the FRS of the dynamic obstacle at time 't' and remove it from the full BRS


# Note:
    For Step 4b, computing the 1-step BRS will result in a conservative under-approximation of the actual safe space. That is because,
    if we wanna obtain the exact safe space resulting from that interaction, we need to compute all the states that INEVITABLY will take
    us in that unsafe state. However, computing the 1-step BRS will give us all the states from which is possible to end up in that unsafe state.

    Therefore in order to compute only the states that will INEVITABLY take us in that unsafe state, we need to compute the 1-step BRS






# Finding a way to compute all the states that will inevitably get you in the obstacle

# Step 1:
First compute the BRS from the obstacle

# Step 2:
Then compute BRS from the complement of the obstacle. This show all the states such that there exists at least one input that will
get you into a safe state.

Maybe we don't even need to compute the BRS from the obstacle, maybe we just need Step 2. More precisely we need to compute the 
n-step BRS from the complement of the n-step FRS of the moving obstacle. Then we compute the intersection of that BRS
with the BRS we previously computed starting from the parking spot (ego vehicle BRS).








# Main Loop:

# Step 1:
    - Compute the n-step brs from the parking spot (brs_n)

# Step 2: # Finding the static image of the obstacle
    - Compute the n-step frs from the non-ego vehicle (frs_n)

# Step 3: 
    - Compute the commplement of frs_n compl(frs_n)
    - Compute the intersection of compl(frs_n) and the entire ego-vehicle state space  inters(compl(frs_n), ss)
    
    - Compute the n-step brs from inters(compl(frs_n), ss) (safe_n)

    - Compute the intersection of safe_n with safe_1_(n-1)  safe_full_n

# end of loop

- Compute the intersection of safe_1_full_n with the full brs (union of all brs_n) space





## Test 1:
 - Setup an example where we know that the full brs_n is the full space













# Improvements:

    # Step 1:
        - Reduce the dimensions of the state space to account for the fact that the ego-vehicle is not a point mass




# Ideas - Results:
    - Add multiple ego and non-ego vehicles
    - Compute the full BRS space without any obstacles
    - Remove the non-safe space due to static obstacles
    - Remove the non-safe space due to dynamic obstacles
    - Perform model checking for each ego vehicle
        - Set membership for each ego vehicle
        - If ego vehicle belongs in safe space then it is safe
        - If ego vehicle belongs does not belong in safe space then it is not safe
    - Manually move all ego and non-ego vehicles one step forward
    - Repeat the full process





















1. Describe in maximum 500 characters why you are interested in this particular Doctoral student-position and project?

While working on my MSc thesis project I've been exposed to the use of formal methods for guaranteeing safety for autonomous systems under complicated tasks described using the temporal logic formalism. During that time I developed a great interest in the notion of formally providing guarantees for robotic and autonomous systems in general and I would like to extend my knowledge in this field. I believe that the PerCoSo project and in particular this Doctoral position will allow me to do so.


2. Describe in maximum 500 characters what you think you can contribute with to the project and the research group.

- Recent experience with formal methods for autonomous systems
- Research oriented mindset developed during my previous academic research projects contributing to the publication of two papers
- My Electrical Engineering background combined with my exposure to practical experience developed through work with the FoilCart project
  gives me the necessarry skills to approach the problem from both a theoretical and practical perspective


I believe that my previous academic research projects and my recent experience with formal methods for autonomous systems will allow me to contribute to the project and the research group. Furthermore, my Electrical Engineering background combined with my exposure to practical experience developed through work with the KTH FoilCart project gives me the necessarry skills to approach the problem from both a theoretical and practical perspective.



My research background in formal methods for autonomous systems alongside previous academic experience equips me to provide valuable insights into the project. I plan to leverage my experience from the KTH FoilCart project to bridge theoretical concepts with practical implementation. With a blend of robotics expertise and hands-on problem-solving, I'm prepared to contribute to innovative solutions that integrate theory and real-world application.



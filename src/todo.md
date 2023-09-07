

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













# Code structure
    - Environment:
        - Static Obstacles
        - Targets
            - Safety
            - Reach Parking Spot
            - Exit Space
        - Cars
            - Car 1
                - State Space
                - Dynamics
                - Initial State
            - Car 2
                - State Space
                - Dynamics
                - Initial State
            - Car 3
                - State Space
                - Dynamics
                - Initial State
            ...


# Todo:
    - Compute the full BRS for the new state space
        - Use this to ensure that the full state space is inside the BRS (39 steps in total are needed to cover the entire space)

    - Compute the FRS for each car
        - Think of how many steps you need to compute the FRS for
            - One idea is to keep record of how many steps you needed to compute the BRS to cover the entire state space
                - However, that might be way too expensive
                - This metehod however, is not going to give guarantees cause for the FRS we need over-approximation and this will be under-approximation
            

            - Compute FRS until all agents safety status is determined
                - Status 1: Agent can safely exit the parking lot
                - Status 2: Agent is not safe
                - I am not sure if this method can be rigorous or not

            - Think of assumptions you can make on the dynamic obstacles that would allow you to set a limit in the number of FRS steps and still compute the FRS and provide guarantees rigorously
                - One idea is to see if you can prove that if you have an 'n' number of cars and 'm' number of roads or intersections, there is always a way for the cars to be safe.
                    - However, I do not like this idea.
                


            - Actually, it should converge, cause there are alternative paths for the ego car to follow. Just take a turn prior to that.
                - This is gonna give true guarantees.
                - You need to mention that as long as the influence of the dynamic obstacle converges
                - Now you just need to experimentally determine the number of steps to do so.
                - Alternatively, maybe there is some smart and computationally efficient way to check for the convergence.
                    - Maybe over-approximate as CZ, compute its comeplement and check if its intersection with... is empty or not

                    - If intersection between the complement of FRS_{t-1} and FRS_t is empty, then the influence of the dynamic obstacle has converged


                - How can I prove that the example I am now working on will indeed converge?






    - For each intersection define a dictionary containing the space right before it, and the state space of interest to it.
        - State Space: Hybrid Zonotope
        - coming: integer

    - Each car object will have a variable that will keep track of the intersection it is currently in
        - integer

    - For each car object its method defining its state space will simply read the integer value of the road it is in right now and return the state space from the equivalent intersection dictionary.
        - For now keep this state space separate from the full state space method.


    - Define a class containing definitions for all parts of the road and all intersections
        - This can be used to automatically detect in which region the center of each car is in so that we can assign the appropriate 'coming' number to it


    - IMPORTANT:
        - Define the conflicting areas in the map (road intersections).
        - Compute the intersection between the FRS and the road intersection area (elarge the area).
            - If the intersection is empty then there are no unsafe areas imposed by that dynamic obstacle at that time step in the future.
            - If the intersection is not empty then:
                - Overapproximated as a hypercube constrained zonotope their intersection and continue with the normal process.

    - NEW IDEA:
        - Only compute the unsafe space induced by cars that are in the 'coming' region of the dangerous intersections.

    - FIX:
        - Fix the issue with 'current_road' for intersections where cars can come from multiple roads



    - Red roads: Indicate all the 'current_roads' of importance that a car could be in at time zero.
    - Red intersection: Closest intersection (in the FRS set)
    - Green intersection: Intersection that might affected by the FRS but later on.
    - Each read 'current_road' should also include the red intersection
    - When initializing the FRS state space, conflict space, ..., for the cars run a loop checking if the center of the car Hybrid zonotope is inside the 'current road' of each intersection


- TODO : 27/08/2023
    - Change all intersection methods to create a list of all conflict areas.
    - Add different bounds for each conflict zone



- TODO: 03/09/2023:
    - Automate initial state of cars:
        - Center of zonotope
        - Generator matrix of zonotopes (take the angle into consideration both for position and velocity)
    - Include the static obstacle (rocks in the street)






1. Describe in maximum 500 characters why you are interested in this particular Doctoral student-position and project?

While working on my MSc thesis project I've been exposed to the use of formal methods for guaranteeing safety for autonomous systems under complicated tasks described using the temporal logic formalism. During that time I developed a great interest in the notion of formally providing guarantees for robotic and autonomous systems in general and I would like to extend my knowledge in this field. I believe that the PerCoSo project and in particular this Doctoral position will allow me to do so.


2. Describe in maximum 500 characters what you think you can contribute with to the project and the research group.

- Recent experience with formal methods for autonomous systems
- Research oriented mindset developed during my previous academic research projects contributing to the publication of two papers
- My Electrical Engineering background combined with my exposure to practical experience developed through work with the FoilCart project
  gives me the necessarry skills to approach the problem from both a theoretical and practical perspective


I believe that my previous academic research projects and my recent experience with formal methods for autonomous systems will allow me to contribute to the project and the research group. Furthermore, my Electrical Engineering background combined with my exposure to practical experience developed through work with the KTH FoilCart project gives me the necessarry skills to approach the problem from both a theoretical and practical perspective.



My research background in formal methods for autonomous systems alongside previous academic experience equips me to provide valuable insights into the project. I plan to leverage my experience from the KTH FoilCart project to bridge theoretical concepts with practical implementation. With a blend of robotics expertise and hands-on problem-solving, I'm prepared to contribute to innovative solutions that integrate theory and real-world application.




















1185, 350       Top left
4720, 2600      Bottom right

Width: 3535
Height: 2250





******************************************************************
t = 1
ng = 5119
nc = 4165
nb = 954
tc = 53.95105682499707
tv = 214.77636027202243
******************************************************************

******************************************************************
t = 2
ng = 4235
nc = 3447
nb = 788
tc = 43.94182726100553
tv = 178.6048499449971
******************************************************************

******************************************************************
t = 3
ng = 4459
nc = 3629
nb = 830
tc = 46.36652074102312
tv = 184.4476452220115
******************************************************************














######################################################

******************************************************************
t = 1
ng = 11303
nc = 9189
nb = 2113
tc = 312.1176412689965
tv = 597.9043564160238
******************************************************************
******************************************************************
t = 2
ng = 10610
nc = 8626
nb = 1983
tc = 282.3234527420136
tv = 549.8059752390254
******************************************************************
******************************************************************
t = 3
ng = 9427
nc = 7665
nb = 1761
tc = 248.4901699979673
tv = 476.9060481970082
******************************************************************
******************************************************************
t = 4
ng = 15628
nc = 12706
nb = 2919
tc = 556.9120671729906
tv = 982.0229159989976
******************************************************************
******************************************************************
t = 5
ng = 15031
nc = 12221
nb = 2807
tc = 542.3801531220088
tv = 942.0766711179749
******************************************************************
******************************************************************
t = 6
ng = 17503
nc = 14229
nb = 3272
tc = 500.1426282579778
tv = 1138.8982621710165
******************************************************************
******************************************************************
t = 7
ng = 17108
nc = 13908
nb = 3198
tc = 486.9540441350546
tv = 1084.7996328870067
******************************************************************
******************************************************************
t = 8
ng = 16606
nc = 13500
nb = 3105
tc = 300.43900853797095
tv = 1054.2310451130033
******************************************************************
******************************************************************
t = 9
ng = 14783
nc = 12019
nb = 2763
tc = 253.10443346598186
tv = 910.9289228579728
******************************************************************
******************************************************************
t = 10
ng = 13983
nc = 11369
nb = 2613
tc = 213.53130968898768
tv = 848.638991559972
******************************************************************





# Finding obsolete backward reachable sets:
    - Step 0: Setup
        - Define related_conflict_area = ...
        
        The related_conflict_area is the area right before entering the other intersection that is the breakpoint of the non-safe space induced due to the dynamic obstacle.

        - It is important to emphasize again that related_conflict_area is NOT the other intersection related to it, but the one step_size position area
        before entering the other intersection, from the side between the two intersections.

    - Step 1: Compute the obstacle n-step FRS
    
    - Step 2: Check if it intersects with the conflict area

    - Step 3: Compute the complement of the intersection of the obstacle n-step FRS and the conflict area

    - Step 4: Compute the n-step BRS from the complement of the intersection of the obstacle n-step FRS and the conflict area
        - Check if the intersection of related_conflict_area and the n-step BRS is empty
            - If it is empty then the n-step BRS and all following k-step FRS and BRS, where k > n are obsolete as well
            - Else continue normally





# Fix car1 and car4





# Car 2:
******************************************************************
t = 1
ng = 3053
nc = 2481
nb = 570
tc = 8.624911948980298
tv = 171.9411649319809
******************************************************************
******************************************************************
t = 2
ng = 3181
nc = 2585
nb = 594
tc = 9.530995209002867
tv = 183.0832493340131
******************************************************************
******************************************************************
t = 3
ng = 3000
nc = 2438
nb = 560
tc = 10.032973836991005
tv = 172.745537759969
******************************************************************
******************************************************************
t = 4
ng = 3160
nc = 2568
nb = 590
tc = 11.369623761973344
tv = 180.99784057500074
******************************************************************
******************************************************************
t = 5
ng = 3320
nc = 2698
nb = 620
tc = 12.122979037987534
tv = 186.06544840801507
******************************************************************
******************************************************************
t = 6
ng = 5175
nc = 4205
nb = 968
tc = 18.22634263895452
tv = 266.1578743000282
******************************************************************
******************************************************************
t = 7
ng = 4791
nc = 3893
nb = 896
tc = 15.8921819619718
tv = 280.89954154298175
******************************************************************
******************************************************************
t = 8
ng = 4354
nc = 3538
nb = 814
tc = 14.061160065000877
tv = 257.273652939999
******************************************************************
******************************************************************
t = 9
ng = 3800
nc = 3088
nb = 710
tc = 13.14987685799133
tv = 219.46973306802101
******************************************************************
******************************************************************
t = 10
ng = 3448
nc = 2802
nb = 644
tc = 11.794155874988064
tv = 198.50759494502563
******************************************************************
******************************************************************
t = 11
ng = 3096
nc = 2516
nb = 578
tc = 10.545305671053939
tv = 177.71563806803897
******************************************************************
******************************************************************
t = 12
ng = 2744
nc = 2230
nb = 512
tc = 9.362084535008762
tv = 155.34381197800394
******************************************************************





# Car 3:
******************************************************************
t = 1
ng = 3320
nc = 2698
nb = 620
tc = 10.975450783967972
tv = 196.16815589298494
******************************************************************
******************************************************************
t = 2
ng = 5175
nc = 4205
nb = 968
tc = 17.370237470022403
tv = 305.55741359398235
******************************************************************
******************************************************************
t = 3
ng = 4738
nc = 3850
nb = 886
tc = 15.806614339991938
tv = 278.27151075104484
******************************************************************
******************************************************************
t = 4
ng = 4333
nc = 3521
nb = 810
tc = 15.16556400997797
tv = 252.3362887200201
******************************************************************
******************************************************************
t = 5
ng = 3949
nc = 3209
nb = 738
tc = 14.494209672033321
tv = 228.90438630298013
******************************************************************
******************************************************************
t = 6
ng = 3565
nc = 2897
nb = 666
tc = 13.295993657025974
tv = 205.5318112160312
******************************************************************
******************************************************************
t = 7
ng = 3181
nc = 2585
nb = 594
tc = 8.514656397979707
tv = 175.0067137160222
******************************************************************
******************************************************************
t = 8
ng = 2797
nc = 2273
nb = 522
tc = 7.4654042609618045
tv = 155.39972132898401
******************************************************************
******************************************************************
t = 9
ng = 2413
nc = 1961
nb = 450
tc = 6.8064510760013945
tv = 134.27795535197947
******************************************************************
******************************************************************
t = 10
ng = 2040
nc = 1658
nb = 380
tc = 5.822543579968624
tv = 113.40500283503206
******************************************************************
******************************************************************
t = 11
ng = 1699
nc = 1381
nb = 316
tc = 5.517867515969556
tv = 93.33281315496424
******************************************************************
******************************************************************
t = 12
ng = 1390
nc = 1130
nb = 258
tc = 4.27762922499096
tv = 75.90061348001473
******************************************************************



# Car 4:
******************************************************************
t = 1
ng = 11465
nc = 9315
nb = 2148
tc = 55.12372765300097
tv = 732.9983649850474
******************************************************************
******************************************************************
t = 2
ng = 10708
nc = 8700
nb = 2006
tc = 51.00199352996424
tv = 679.0630720179761
******************************************************************
******************************************************************
t = 3
ng = 9983
nc = 8111
nb = 1870
tc = 48.38442691997625
tv = 633.5239784240257
******************************************************************
******************************************************************
t = 4
ng = 4407
nc = 3581
nb = 824
tc = 16.271400440018624
tv = 243.75608245900366
******************************************************************
******************************************************************
t = 5
ng = 3970
nc = 3226
nb = 742
tc = 14.716054284013808
tv = 218.89818242401816
******************************************************************
******************************************************************
t = 6
ng = 3000
nc = 2438
nb = 560
tc = 8.933577543997671
tv = 146.45834448601818
******************************************************************
******************************************************************
t = 7
ng = 2659
nc = 2161
nb = 496
tc = 7.911375709983986
tv = 133.14472081599524
******************************************************************
******************************************************************
t = 8
ng = 3000
nc = 2438
nb = 560
tc = 8.929470052942634
tv = 166.33971224504057
******************************************************************
******************************************************************
t = 9
ng = 3160
nc = 2568
nb = 590
tc = 10.241531053965446
tv = 179.6062474520295
******************************************************************
******************************************************************
t = 10
ng = 3320
nc = 2698
nb = 620
tc = 12.079515933990479
tv = 188.1085944530205
******************************************************************
******************************************************************
t = 11
ng = 5175
nc = 4205
nb = 968
tc = 21.04591637995327
tv = 289.14383293199353
******************************************************************
******************************************************************
t = 12
ng = 4738
nc = 3850
nb = 886
tc = 18.602289789007045
tv = 263.4036922919913
******************************************************************



# Car 5:
******************************************************************
t = 1
ng = 3181
nc = 2585
nb = 594
tc = 10.896986076957546
tv = 149.48527518700575
******************************************************************
******************************************************************
t = 2
ng = 2797
nc = 2273
nb = 522
tc = 9.576940800994635
tv = 134.02313213801244
******************************************************************
******************************************************************
t = 3
ng = 2413
nc = 1961
nb = 450
tc = 6.193656635994557
tv = 120.0784592150012
******************************************************************
******************************************************************
t = 4
ng = 2040
nc = 1658
nb = 380
tc = 5.259524891967885
tv = 102.04780651297187
******************************************************************
******************************************************************
t = 5
ng = 1699
nc = 1381
nb = 316
tc = 4.537445918016601
tv = 83.55561717000091
******************************************************************
******************************************************************
t = 6
ng = 1390
nc = 1130
nb = 258
tc = 3.7962031469796784
tv = 67.81698761996813
******************************************************************
******************************************************************
t = 7
ng = 1113
nc = 905
nb = 206
tc = 2.7142406050115824
tv = 48.863841366022825
******************************************************************
******************************************************************
t = 8
ng = 868
nc = 706
nb = 160
tc = 1.9636382189928554
tv = 36.82082975801313
******************************************************************
******************************************************************
t = 9
ng = 655
nc = 533
nb = 120
tc = 1.5330384370172396
tv = 27.663153941975906
******************************************************************
******************************************************************
t = 10
ng = 474
nc = 386
nb = 86
tc = 1.1443144470104016
tv = 20.10910571500426
******************************************************************
******************************************************************
t = 12
ng = 325
nc = 265
nb = 58
tc = 1.2538604579749517
tv = 13.929719868989196
******************************************************************

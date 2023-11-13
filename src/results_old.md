# Our Approach
    - Introduction
        - In this chapter we layout our approach to solving the problem of generating a temporal logic tree to fascilitate real time model checking and formal control synthesis \todo{FIX THIS}.

    \Section{Plant}
        - Plant dynamics, and other environment assumptions
                
    \section{Case Study 1: Static Environment}
        - Introduce the static environment
        - State any assumptions made specifically for this environment
        - Clearly state the abstract goal of this case study
        - Construct the LTL specification from that abstract goal
        - TLT
        - Provide information about the computation of the different set nodes of the TLT
        - ???
    \section{Case Study 2: Dynamic Environment}
        - Introduce the dynamic environment
        - State any assumptions made specifically for this environment
        - Clearly state the abstract goal of this case study (Environment TLT, ...)
        - Constructing the LTL and TLT
            - In this case study the TLT changes over time, however, its structure remains the same, thus, we first provide and explain the general structure of the TLT here, and then we explain how we need to recompute its set nodes over time

            - Structure of the TLT
                - In this part we provide the structure of the TLT.
                - First explain how we would deal with a static obstacle (always operator and then compute the RCI of the static obstacles)

                - However, in this case since we are dealing with moving obstacles, merely asking to always stay outside of the trajectory of the moving obstacle would result in a very conservative solution, thus, we now introduce a new idea of computing the unsafe regions for each time step separately. The general idea is that we predict the trajectory of the moving obstacles by computing their forward reachable sets and then for each time step 'k' in the prediction horizon we compute the regions that would inevitably collide with the k-step forward reachable set of the obstacles.
                - So we now provide an alternative LTL and TLT
                - Thus, we now introduce the idea of staying outside of the non-safe region $\mathcal{U}_{\mathcal{O}}$ region of the dynamic environment,
            
                - Then we need to provide an example of how this idea works. Use the environment you created in the Preliminaries chapter and put two cars approaching an intersection
                - Provide an example: (use the environment you created in the Preliminaries chapter)
                    - First picture:
                        - One car approaching an intersection, then color the safe and unsafe regions introduced by that car.
                    - Second picture: 
                        - In this picture we want to show that indeed the set computed correctly shows the safe and unsafe regions
                        - In this picture we add two more cars on the side of the intersection affected by this car.
                            - One of them in the safe and one in the unsafe region
                    - Third picture:
                        - Move all three cars to verify that the prediction is correct

                - Now it is time to provide a formal description of this, where we show how we compute the safe region as well as the unsafe region $\mathcal{U}_{\mathcal{O}}$.

            - Computing Set Nodes
                - Here you will explain our new approach for computing the safe set

        - Explain how we can handle moving obstacles
            - Here you might wanna introduce the idea of computing the RCI of moving sets (Ask Frank about how you should do this)

////////////////////////////////////////////////////////////////////////////////////////////////

# Our Approach

## Show an example of the computation of the entire 

## RCI for Hybrid zonotopes
    - Provide an algorithm for it
    - Show examples:
        - First start with a static object
        - Then move to a moving object
            - Ask Frank if you can term it as "moving RCI" or something like that in case this idea does not already formally exist
        - 2D or 3D?


3. Our Approach
    - In this chapter we start by describing the scenario we are gonna deal with later on in the expirements chapter as well.


3.1 LTL specification
    - Explain how each LTL component is mapped into the TLT

3.2 Generation of temporal logic tree
    - Explain how each set node of the TLT is computed

3.3 RCI for Constrained Zonotopes / Hybrid Zonotopes

3.4 Dynamic Environment
    - Introduce the dynamic environment
    - Update the LTL specification to match the new goals
    - Explain how we can handle moving obstacles
        - Here you might wanna introduce the idea of computing the RCI of moving sets (Ask Frank about how you should do this)



# Our Approach

## Show an example of the computation of the entire 

## RCI for Hybrid zonotopes
    - Provide an algorithm for it
    - Show examples:
        - First start with a static object
        - Then move to a moving object
            - Ask Frank if you can term it as "moving RCI" or something like that in case this idea does not already formally exist
        - 2D or 3D?




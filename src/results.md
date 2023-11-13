# Results

## Static Environment

    - For the static environment we use the same environment and conditions as in the previous chapter, however, we will once again provide the formula and the environment it self here for completeness. Given the goal of eventually reaching one of the available parking spots in the environment shown in Figure~\ref{}, we construct the LTL formula (\ref{}) and the TLT shown in Figure~\ref{}. At this point it is important to stress how no obstacles are directly taken into accoung in the LTL formula. This is done because we know that the environment is static and we can simply define the state space in such a way to not include the obstacles. In addition, to take into account the dimensions of the ego-vehicle, we further inflate the obstacles accordinigly a prior to defining the state space.

    - LTL formula

    - Environment and TLT

    - Results
        - A couple of screenshots of the simulation
        - Put all screenshots in one figure
        - Comment on the interesting parts of these screenshots collectively, while also directing the reader to a specific screenshot if needed
        - Explain the story of how this is simply the process of computing the root node of the TLT, and that the TLT structure allows us to compute the BRS for all parking spot independently, and to do so in parallel.

## Dynamic Environment

    - Now it is time to move the dynamic environment case. In this case we extend the environmet and the LTL to include multiple moving obstacles, more precisely, we include five non-ego vehicles navigating around the parking lot as shown in Figure~\ref{}.
    
    - Environment
    
    The goal now is to navigate around the parking lot towards one of the two exits while avoiding collisions with any other vehicles. The LTL formula for this goal is shown in (\ref{}) and the TLT is shown in Figure~\ref{}.
    
    - LTL formula
    - TLT

    - Results







# NEXT STEPS:
    - Use chatgpt for rephrashing
    - Change template for thesis


# TODO:
    - Tickets
    - Email Ivan an Nick
    - THS
    - Rent
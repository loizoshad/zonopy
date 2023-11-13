



# Slide 1: Introduction

- Script:

    In the last decades we've seen tremendous advancements in ...

    Sometimes we need to take a step back and see if we are on a safe path.

    Common examples where people are concerned are, autonomous vehicles, AI, and even workspaces shared by machinery and people.

    What these three examples have in common, is people not having faith in how these machines will operate/respond, because they deem them unpredictable. 

    My goal is not to answer when these beliefs are justified, instead our work here is to aid in providing the necessarry tools to guaranteed that the machinery's behavior will not be unpredictable, and so it will indeed be deemed worth of people's trust.

- Slide Contents:
    #
    - High complexity and need for reliability
    #

    - Developments in compute science, hybrid systems, and cyber-physical systems have led to increasingly complex systems.
    - Need for reliabile and robust systems

    - Three images:
        (HOW SHOULD I PROPERLY CITE THESE IMAGES?)
        - One image for autonomous vehicles or smart city (use the one you already have in the presentation)
        - Robotics
            - Find one that shows people in the loop
        - AI-based solutions
            - Safe training of RL agents


# Slide 2: Introduction

- Script:

    This pursuit naturally introduces the need for adept methods both for describing the systems' and its goals' complexity, understanding them and ultimately utilizing them to our own benefit.

    We approach this problem through the lenses of formal methods, where one is called upon to rigorously provide guarantees about the behavior of a system.

    An appealing idea is to utilize the temporal logic formalism whose expressiveness allows us to capture a system's temporal behavior in a compact way.

    Then we use reachability analysis, a tool that enables the analysis of robustness properties by providing insights about the possible behaviors of a system.

    More precisely, in our work we will first define 


- Slide Contents:
    #
    - So we need methods that can take care of these complex tasks
        - Some approaches use temporal logic, reachability analysis, or a combination of them
    #

    - Left side: Temporal logic figure and title:
        - Task: "Eventually" reach the exit ..., and "Always" stay safe.
    - Right side: Reachability analysis figure and title:
        - Overtaking scenario, where you 
    
        - Add a road environment, with cars approaching an intersection, and show how the ego vehicle can either choose to go the intersecetion and collide, or
        simply turn elsewhere and avoid the collision

    


# Slide 3, 4: Automata Vs TLT       DONE
- Slide Contents:
    #
    - The overwhelming majority makes use of the theory of automata to achieve that...
       - Explain the full theory
    - An alternative to automata was recently introduced suggesting the use of a tree structure called TLT which encodes information both about the goal and the states that can satisfy the goal. This tree structure resolves this issues as it is abstraction free for infinite systems...
    #


# Slide 5: Goals/Contributions      DONE
- Slide Contents:
    #
    - So, in this work we utilize the expressiveness of linear temporal logic, and construct the aforementioned tree structure through reachability analysis.
      At the same time we accelerate the reachability analysis, by utilizing the computational efficiency of a new set representation introduced in 20..., that is the Hybrid Zonotope.
    - Ultimately, our contributions are:
        - Due to the accelerated computations with Hybrid Zonotopes, we can provide guarantees for dynamic environments
        - Extend the applicability of Zonotope-like approaches to complex tasks
            - Typically, zonotope-like approaches are only applicable to simple tasks
        - ...
    #

# Slide 5: Background: Reachability and Invariance      DONE

- Slide Contents:
    #
    - Background: Reachability
    #

    - Include FRS as well
    - Perhaps add the images you have in your thesis report


# Slide 6: Background: Temporal Logic                   DONE
- Slide Contents:
    #
    - Background: LTL & TLT
        - Provide that has a TLT as well
    #

    - Theory for LTL and TLT
    - Example goal, LTL, and the generated TLT as well

# Slide 7: Background: Set Representation
- Slide Contents:
    - Multiple set representations.
    - Zonotopes slide, both definitions and example sets.

####
Now I will proceed to explain our method while also showing our results at the same time

# Slide 8: Case Study: Static Enviroment

# Slide 9: Case Study: Dynamic Environment

Alternatively for the case studies you can:

    - Ignore static environment now. Instead create a slide for "Methodology", where you slowly explain the idea of how the TLT is generated for  simply case of lets say reaching one target (1 parking spot)
    
    - Then proceed to study how you can deal with obstacles and for that, follow the structure in your thesis report.


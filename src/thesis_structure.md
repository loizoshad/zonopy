# Introduction

Developments in fields like computer science, hybrid systems, and cyber-physical systems have ushered in a new era of complexity. As we continue to push the boundaries of what is possible, we are faced with the challenge of controlling ever more intricate systems all while providing reliability and robustness. This pursuit naturally introduces the need for adept methods both for describing their complexity but also analyzing it and understanding it. Both facets are of great significance as they constitute fundamental building blocks for the verification and validation of a system's behavior.

In this report, we explore the use of temporal logic as a formalism for describing dynamic systems influenced by temporal behaviors. We then turn our attention to reachability analysis, a tool that enables the analysis of robustness properties by providing insights about the possible behaviors of a system. We then explore the use of reachability analysis in the context of real-world tasks described using linear temporal logic specifications.



## Background

Reliably controlling dynamical systems has been a topic of great interest for many years. The need for robustness and reliability is especially important in the context of safety-critical systems.

### Reachability Analysis
    - A typical way to provide such guarantees is to use reachability analysis tools [REFERENCE FOR WORK ON REACHABILITY ANALYSIS: MAYBE REFERENCE A TEXTBOOK]. Given that we have a model
    of the system and a description of the task we want to achieve, reachability analysis can be used to determine which set of states of the system can
    satisfy the task.

    - However, the complexity of the system and the task we want to achieve have a significant impact on the computational cost of the reachability analysis.
    More specifically, this complexity is generally reflected in the dimensionality of the system to which reachability analysis do not scale well.

    - Therefore, one idea is to abstract away this complexity by using a formalism that allows us to describe the system and the task in a compact way, but
    at the same time fascilitates the reachability analysis upon it. This is where temporal logic comes into play.

    - Temporal logic is a formalism that allows us to describe the behavior of a system over time. It is a very powerful tool that allows us to describe
    complex behaviors in a compact way.
    
    - The idea is to use temporal logic to describe task and then create a structure on which reachability analysis can be performed.


### Automata for Temporal Logic

    - The overwhelming majority of the work in this field focuses on the use of automata-based methods [jiang2020ensuring 17], [jiang2020ensuring 18]. The idea is to describe the system as a finite state machine and then use the automata to check whether the system satisfies a given specification. The specification is also described as a finite state machine.

    - Althoough this approach is very powerful, it does however, come with some inherent restrictions when it comes to infinite systems.

    - The process of abstracting infinite systems to finite systems is not trivial. Abstraction techniques generally partitions the state space and then constructs transitions through reachability analysis. The computational complexity of this process increases exponentially with the dimensionality of the system. Although many works focus on enhancing its computational efficiency, such as through over-approximation [gao2021temporal 25], or by leveraging the underlying uncertainty's structure [gao2021temporal 22], they are still limited in their generality. This is because abstraction techniques are not generally applicable, instead they viable for systems with incremental stability, smooth dynamics, etc.

    - In addition automaton-based solutions generally only support a subset of the temporal logic language, such as the bounded LTL or co-safe LTL [gao2021temporal 39], [gao2021temporal 40]. These restrictions stems the conservative overapproximation of forward reachable sets which is widely used for the abstraction process and can lead to significant loss of information.

    - Finally, existing approaches often face limitaations in terms of online scalability. Assuming perfect knowledge about a given specification is not always accurate enough to rely on. Typical examples include problems where multiple agents are involved and their behavior is not known in advance, neither can it be controlled. In such cases, it is not possible to define a specification that captures all the possibilities during the navigation process. As a result, the capability to design automaton-based methods offline is severely constrained. Therefore, one needs to resort to methods that fascilitate online implementation,
    which automaton-based methods do not as they lack modularity its nodes are heavily dependent on each other.
    

### Temporal Logic Trees for Temporal Logic

    - A promising alternative first proposed in (gao2021temporal) is the construction of a tree structure called Temporal Logic Tree (TLT) which simultaneously encodes the LTL specification, but also which set of states satisfy the specification. This is done by constructing a tree structure where each node. A TLT is a tree whose nodes are either temporal (and boolean) operators, or set nodes computed via reachability analysis. In contrast to the automata-based methods, TLTs are abstraction free for infinite systems, they can handle the full LTL language, and they permit online implementation. An immediate result from structure, TLTs allow to break down one complex reachability analysis problem into a series of smaller ones, which can also be computed in parallel, and replaced in an online-fashion due to its high modularity.


    - In [\todo{jiang2020ensuring}] the authors employ the TLT structure to ensure the safety for vehicle parking tasks using Hamilton-Jacobi reachability analysis to construct the tree. In their work, the authors successfully generate the TLT and update it online. However, the updates in the tree are only limited to minor changes of new parking spot being available on run-time, but they do not handle cases where other vehicles are present in the environment which would ultimately require predicting their behavior online. This restriction is heavily attributed to the use of Hamilton Jacobii reachability analysis.




- Describe the process of first computing the TLT, model checking, control synthesis...
Also, this is the point where you show that in this work we are only focusing on the generation of the temporal logic tree, which can later on be
used for model checking and control synthesis.

- TODO: Find a good motivation as to why we only focus on that part instead of the whole process.


## Contributions

In this work we focus on the generation and update of the temporal logic tree in an online fashion, which can also later on allow one to perform online model checking and formal control synthesis. More precisely, given a task described by a linear temporal logic formula, the task is then translated into a temporal logic tree (TLT) whose structure will also change during run-time based on the changes within the environment. The computation of all set nodes of the TLT is then performed using reachability analysis based on hybrid zonotopes [bird2021hybrid]. Hybrid zonotopes benefit from the high computational efficiency of zonotopes, but also allow for the representation of non-trivial sets as it is generally the case for zonotope-based solutions. Thus, in this work we make the following contributions:

- We provide real-time robustness guarantees for temporal tasks
- We fascilitate real-time model checking and control synthesis for a given task
- We contribute to the unification of temporal logic and reachability analysis
- We extend the applicability of zonotope-based representations to complex problems

## Outline






[//]: #######################################################

# Preliminaries

## Notation

## Plant Model

## Temporal Logic
- Linear Temporal Logic
- Temporal Logic Trees

## Reachability Analysis
    - Forward Reachability (FRS)
    - Backward Reachability (BRS)
    - Invariant Sets

## Set Representation
    - Level Sets
    - Polytopes
    - Zonotopes
    - Constrained Zonotopes
    - Hybrid Zonotopes


[//]: #######################################################

# Our Approach

## Show an example of the computation of the entire 

## RCI for Hybrid zonotopes
    - Provide an algorithm for it
    - Show examples:
        - First start with a static object
        - Then move to a moving object
            - Ask Frank if you can term it as "moving RCI" or something like that in case this idea does not already formally exist
        - 2D or 3D?



































1. Introduction

1.1 Background
1.2 Contributions
1.3 Outline


2. Preliminaries

2.1 Notation
2.2 Plant Model
2.3 Reachability Analysis

2.4 Temporal Logic
    - Linear Temporal Logic
    - Temporal Logic Trees
        - Since we are only dealing with the computation of the TLT and its update in an online fashion, we should emphasize more on the 
        generation of the TLT and its real time capabilities, instead on the model checking and control synthesis.
        However, you should still mention that the TLT will then be used to perform model checking and control synthesis.

        
        - To illustrate the idea of the TLT, we consider briefly explain an example of how to construct a TLT for a given LTL specification regarding parking of autonomous vehicles. Consiser 

        - In this work we emphasize first on the construction of a static TLT as shown , and then 

        - Generation
        - Model Checking
        - Control Synthesis

2.5 Set Representation
    - Level Sets
    - Polytopes
    - Zonotopes
    - Constrained Zonotopes
    - Hybrid Zonotopes



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



4. Results

4.1 Static Environment
    - Focus on showing the capabilities of the Hybrid zonotope in terms of teh complexity of the environment.
    - Try to showcase the computation of the safe space due to all the static obstacles in the environment
        - This can be used as a good enough argument in the dynamic environment to say that since we know that these objects are static,
        we precompute it.


4.2 Dynamic Environment

    - Motivate why you do not consider the path travelled by a car as a non-safe space
        - To do that you can say how we are now considering a problem where we don't just wanna see where in the place a car can begin at,
        but we also add 5 cars moving in the environment that are at the same time considered as obstacles but also as agents that want to 
        accomplish the temporal logic task.
        - You can also motivate it later, by its future work. One future work is to later reformulate the specification to see if it is possible
        that all agents can accomplish their tasks without colliding with each other, and then perform control synthesis.


    - TODO: Better motivate the particular specification, resulting safe space and generally the safe space of this experiment.










# Our Approach

## Show an example of the computation of the entire 

## RCI for Hybrid zonotopes
    - Provide an algorithm for it
    - Show examples:
        - First start with a static object
        - Then move to a moving object
            - Ask Frank if you can term it as "moving RCI" or something like that in case this idea does not already formally exist
        - 2D or 3D?

























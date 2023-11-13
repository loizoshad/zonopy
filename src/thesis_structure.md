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












# Outline

In the remainder of this thesis, we will first introduce the preliminaries in Chapter 2 and explain the theory behind reachability analysis, temporal logic, 
and Hybrid Zonotopes. In Chapter 3, we lay out our methodology for constructing a TLT using Hybrid Zonotopes and show how we can deal with environments populated
with moving obstacles. In Chapter 4, we formulate two case studies, one in a static and one in a dynamic environment, where we show the construction of the TLT for each case and showcase some intereseting properties of Hybrid Zonotopes. Finally, in Chapter 5, we conclude our work and discuss some future work that can be done in this area.


# Discussions
    - Limitations
        - Highly non-convex spaces
            - While the Hybtid Zonotope representation demonstrates proficiency in handling non-convex disjoint  sets, its efficacy diminishes when confronted
            when it needs to create non-polygonal shapes. This is because the Hybrid Zonotope representation, similarly to its predecessor, the Zonotope, it only uses a linear combination of its generators to represent a set.
        - High levels of redundancy
            - In addition, the Hybrid Zonotope representation suffers from high levels of redundancy in its representation. Although redundancy removal techniques exist and are used in this work further improvements are still needed as they do not fully get rid of it. Certain operations that introduce a lot of redundancy are the \textit{Union} and the \textit{Complement}.
        - Although the representation allows for fast computation of set operations, if a problem requires check for emptiness, or point containment, the computation time increases significantly, that is because these problems are mixed integer linear programming problems, which are NP-hard.

    - Conclusions:
        - Ensuring the safety of autonomous vehicles throughout their entire operational spectrum poses a challenging yet vital task that requires out attention.
        In this work, we have shown how one might use Linear Temporal Logic to formally specify parking and navigation missions in dynamic environments and how one might use Hybrid Zonotope-based reachability analysis to guarantee safety for these missions. Additionally, we illustrate how the divide and conquer nature of the TLT alongside the high computational efficiency of Hybrid Zonotopes enable online adjustments due to changes in the environment.

        - In general, guaranteeing safety for automated vehicles over their entire operation is difficult, but remains an important problem we need to solve. We show in this paper how one might formally specify parking and navigation missions in dynamic environments using Linear Temporal Logic, and how one might use Hybrid Zonotope-based reachability analysis to guarantee safety for these missions. We also show how one enable online changes to the goal with the use of the highly modular tree structure of the TLT.

        - Compare with related work
            - Although most works using Zonotopes in reachability analysis use them due to their notable computational efficiency, very few works, if any handle problems with the level of complexity that we suggest. This higher level of complexity in the problem is enabled through the use of the temporal logic formalism, and the use of Hybrid Zonotopes allows accelerate the computation of the reachability analysis.
            - A similar work done by the authors of [\todo{jiang2020ensuring}] deals a similar problem as we do, providing safety guarantees for an autonomous parking task. However, the use of Hamilton-Jacobi reachability analysis in their work limits the potential for dealing with dynamic obstacles as we do in our work.


    - Future Work
        - Apply this method on more abstract goals, in order to exploit the hybrid nature of Hybrid Zonotopes, instead of using linear systems. One idea could be to implement this on a more abstract level for coordinating different aspects of a smart city, whose complex dynamics can be abstracted away and allow the Hybrid Zonotope to deal with the distributed nature of the problem.

        - 
        

# English Abstract:

- In this report, we introduce a formal approach aimed at providing guarantees regarding the behavior of autonomous systems operating within dynamic environments. Our methodology is presented within the context of ensuring the safety of a vehicle throughout parking and navigation missions. To formally describe the mission, we employ Linear Temporal Logic. The evaluation of specification fulfillment is fascilitated through the construction of a Temporal Logic Tree via reachability analysis. To accelerate the reachability analysis, we leverage the computational efficiency of Hybrid Zonotopes, a set representation method that allows for fast computation of set operations on non-convex disjoint sets. We illustrate how the utilizaation of Hybrid Zonotopes enables the description of complex and abstract tasks, as well as the ability to deal with environments containing multiple moving vehicles. To demonstrate the efficacy of our approach, we present two case studies: one in a static environment, focusing on parking in one of the available parking spots, and another in a dynamic environment, where the objective is to navigate towards the parking lot exit while avoiding collisions with other vehicles. Our results, showcase the ability of our approach to combine the expressiveness of Linear Temporal Logic with the computational efficiency of Hybrid Zonotopes to provide safety guarantees for autonomous systems operating in dynamic environments.


# Swedish Abstract:


- I denna rapport introducerar vi ett formellt tillvägagångssätt med syfte att ge garantier om beteendet hos autonoma system som verkar inom dynamiska miljöer. Vår metodik presenteras inom ramen för att säkerställa fordons säkerhet under parkerings- och navigationsuppdrag. För att formellt beskriva uppdraget använder vi Linjär Temporal Logik. Utvärderingen av specifikationens uppfyllelse underlättas genom konstruktionen av ett Temporal Logik Träd med hjälp av nåbarhetsanalys. För att påskynda nåbarhetsanalysen utnyttjar vi den beräkningsmässiga effektiviteten hos Hybrid Zonotoper, en metod för representation av mängder som möjliggör snabb beräkning av mängdoperationer på icke-konvexa och disjunkta mängder. Vi illustrerar hur användningen av Hybrid Zonotoper möjliggör beskrivningen av komplexa och abstrakta uppgifter samt förmågan att hantera miljöer med flera rörliga fordon. För att påvisa effektiviteten av vår metod presenterar vi två fallstudier: en i en statisk miljö med fokus på parkering på en av de tillgängliga parkeringsplatserna och en annan i en dynamisk miljö där målet är att navigera mot parkeringsplatsens utgång och samtidigt undvika kollisioner med andra fordon. Våra resultat visar förmågan hos vår metod att kombinera Linjär Temporal Logiks uttrycksfullhet med Hybrid Zonotopers beräkningsmässiga effektivitet för att ge säkerhetsgarantier för autonoma system som verkar i dynamiska miljöer.


# Acknowledgements

I would like to thank my supervisors Frank Jiang and Amr Alanwar for their invaluable advice, continuous support, and patience throughout the course of this work. Additionally, I want to express my gratitude to friends and colleagues, Matthew William Lock, Sujet Phodapol, and Kaj Munhoz Arfviddson, for the long discussions and brainstorming sessions that helped me shape my ideas and improve my work. Last but not least, I wish to express appreciation to my family and friends for their understanding and encouragament throughout my academic journey.









































# Introduction (Front page):
    - Hello everyone, our discussion today will revolve around robustness guarantees on temporal missions.

# Slide 2 - Challenges in Autonomous Systems:
    - In the last decades we've witenssed breakthroughs that have undeniably transformed our world in profound ways, but they have also raised important questions regarding safety.
    
    - These concerns are particularly pronounced in domains where humans interact closely with machines. Imagine, autonomous vehicles navigating our streets, or AI systems making critical decisions.

    - Such problems require that we develop reliable and robust systems, and we need do this rigorously, 
    
    - An immediate answer to that are Formal methods
    
    - Naturally formal methods introduce the need for adept solutions both for describing and analyzing such problems.

# Slide 3 - Temporal Logic & Reachability Analysis:
    
    - An appealing idea is to utilize the expressiveness of the temporal logic formalism to capture a system's goal. and then use reachability analysis, to study the system's robustness properties.

    - To better illustrate this idea, consider the example here where the goal is for the red car to eventually reach the exit, while always remaining safe. This goal as we will see later on can be compactly described using temporal logic. Then given that goal, and performing reachability analysis, we see that according to the dynamic of the vehicle and the traffic rules, the red car can either remain in its current lane which will eventually lead it towards the exit in a safe manner, or it can take the risk and attempt to overtake. However, in the latter case we can not guarantee the safety of the vehicle as it is dangerously close to crushing with the blue car. 

# Slide 4 - Automata vs Temporal Logic:
    - As you might already know, the overwhelming majority of literature leverages the theory of automata to analyze such goals. This is a very well established choice when dealing with finite systems, that is systems with discrete state space, and discrete-time.

    - However, things might get a bit shaky when we transition to infinte systems, as automata suffer from the curse of dimensionality, in addition, they usually do not handle the full LTL language, and they lack online scalability.

    - An alternative approach recently introduced are Temporal Logic Trees. This structure is abstraction free for infinite systems, as they retain the independence from explicit time constraints of temporal logic, and in this case linear temporal logic. Additionally, they can handle the full LTL complexity, and permit online implementation due to their higher modularity.
    
    - These properties are the primary factors for choosing TLTs over Automata in this work.
    

# Slide 5 - Method & Contributions:
    - To summarize our approach and contributions, we first utilize LTL to express abstract tasks,
    - Then we construct the TLT through multi-stage reachability analysis,
    - And to accelerate this process we employ a Hybrid Zonotope-based reachability.

    - Our main contributions are first we extend the applicability of zonotope-based solutions to more complex tasks,
    we fascilitate online model checking for TLTs, and we develop a new method to give robustness guarantees in dynamic temporal tasks.
    
# Slide 6 - Reachability and Invariance:
    - To ensure that we are all in the same page, I want to now go through some basic theoretical concepts.

    - Given the general nonlinear dynamics of a system, we can define the forward and the backward reachable sets, as well the maximally invariant set.

    - Intuitively the forward reachable set denotes all the possible future states the system can attain, while the backward reachable set answers the question of where could we begin in the state space and end up in our current state. Finally, for the invariant set, it is the largest set for which we can always stay within

# Slide 7 - Reachability and Invariance - Example:
    - A visual depiction of each of these sets is shown here where, we see how the forward reachable set, covers the streets that a car starting at the position 'I' can end up in, then the backward reachable set, all the states that could lead to 'T', and finally, for the invariant set, we study the case where we want to always ensure that we will avoid collision with the obstacle.

# Slide 8 - Temporal Logic:
    - Moving on to temporal logic, we will be dealing with linear temporal logic, which is a language whose syntax is given here. To give you a rough picture of this language, the rhombus represents the "Eventually" operation, where we ask the question of if we can eventually satisfy some condition, or for the 'square' which asks, if we can always satisfy some other condition. In addition to that we have different logical operators, as well the fundamental block of this formalism; that is the atomic propositions.

    - To briefly define what a TLT is, it is a tree structure consisting of set or operator nodes from the temporal logic syntax, it is constructed from the LTL formula through multi-stage reachability analysis, and it contains information about which states satisfy the specification.

# Slide 9 - Temporal Logic - Example:
    - The concepts might look a bit to abstract, so to solidify these ideas consider the problem where we want to study where in the parking lot can we start and eventually reach one of the available parking spots. This goal can be described using the eventually temporal operator alongside the 'or' logical operator. Then the resulting tree is shown here, where for each parking spot we solve a separate backwards reachability problem and finally combine them to form its root, which indicates which states can satisfy this specification.

# Slide 10 - Set Representation:
    - Since we are dealing with reachability analysis, we need to choose a suitable set represantaion method. The construction of TLTs for meaningful tasks, generally requires us to compute reachable sets, unions and intersections as well as handling non-convex disjoint sets.

    - In this table I a laying out for reference a list of common representation methods, alongside the relevant operations and then shocase which tools can handle such operations. Clearly the only two set representations capable of handling these requirements are only the Hybrid Zonotopes and Level Sets.

    - In this work we proceed with Hybrid Zonotopes, as they partially retain the computational speed advantages of their predecessor the Zonotope.

# Slide 11 - Hybrid Zonotope:
    - To understand the Hybrid Zonotope, let's start by explaining what a plain zonotope is. A zonotope is merely the linear combination of multiple vectors with their coefficients limited between -1 and 1 as opposed to what we usually do in linear algebra. The resulting shape is just a convex polytope who is centrally symmetric.

    - Its immediate extension is the constrained zonotope, which in addition allows for linear equality constraints to be imposed on those coefficients. This results again in a convex polytope, but this time it does not have to be centrally symmetric. An example is shown in this figure colored as blue on top a the equivalent zonotope with no constraints colored with red.

    - Finally, the hybrid zonotope, can be thought of as the union of multiple constrained zonotopes, and can thus represent non-convex and disjoint sets as shown in the figure right here.



# Slide 12 - 
    - Moving on to our methodology, I will first revisit the parking example I introduced earlier. This time though, I want to emphasize on the structure of the TLT.

    - Starting with the very left branch, it consists of two set nods and one operator node.

# Slide 13 - 
    - Moving on to the main pillar of our work, we now study how we provide robustness guarantees in dynamic environments.
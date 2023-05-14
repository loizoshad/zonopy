<!--
# Report

## Introduction

### Generic description of the problem to be solved.
  • Formal verificiation / Model Checking \
  • Control synthesis

### 1.1 Background (What is the story you wanna tell:)

  - Classically automata are the protagonist in model checking and control synthesis for a FINITE TRANSITION SYSTEM and a linear temporal logic \
  - Give some examples of relevant state of the art automaton-based solutions \
  - However, autoamata have some inherent limitations when used for INFINITE SYSTEMS \
  - An alternative was suggested in [gao2021temporal] \
  - Describe the strengths of the TLT \
  - The TLT require approach asks for a mean performing set operations \
  - Commonly used set representations inlcude: \
    - Level Sets: They allow you to..., and you can compute reachable and invariant sets using Hamilton Jacobii methods. \
    - Polytopes: ..

-->



















# Presentation:


## Front page

  ### Script:

  #### Slide 1:
    - The theme I wanna create today is to showcase the utility of formal methods, and what can be done to improve them. To establish this I wanna point out how society has so far been trusting our technology (even with their lives). Now it reached the point where it is more or less taken for granted that we need to keep on providing humankind with everimproving machinery all while increasing the guarantees that they will work as intended.
    
    - For us to not lose the trust of the people we need to be able to provide them systems that we can guarantee they will work as predicted (with some reasonable margins of course).

    - This becomes especially significant when dealing with systems where lives are directly on the line. Be it airplanes, medical devices, autonomous cars, now that they are on the rise. The list goes on and on. 

    As you are already aware on interpretation of this goal, is to split it into to problems; that is the model checking and the formal control synthesis, where we design controllers and study models such as to ensure they abide by certain requirements.





    - Throughout this presentation I will make EXHAUSTIVE use of an autonomous parking scenario to explain our work and showcase our results.


  ### Slide Contents:

  #### Slide 1:
    - Cover page


## Introduction

  ### Script:

  #### Slide 1: Model Checking and Formal Control Synthesis

    - In many problems it suffices to just check properties such as stability and invariance, however, as the demand for complex and safe systems increases, new properties need to be introduced to ensure.

    - The need for methods that rigorously provide guarantees is increased.

    - This challenge can be cast as a twofold problem where we need to solve the problem of model checking and the problem of formal control synthesis.
    
  #### Slide 2: Temporal Logic

    - One school of thought for translating abstract application requirements into the mathematical domain uses temporal logic.

    - One interesting combination that has been introduced from the commputer science is the usage of temporal logic. temporal logic has the distinct characteristic of being able to abstract away from the dynamics and give task related requirements. This allows for a richer analysis than the conventioanl stability and invariance criteria. Merely the study of stabiliy does not cut it any more.

    - The interpretation of the model checking and control synthesis problem we just assigned to our problem is very abstract and vague. So we now need to express is using a mathematical language.

  ### Slide Contents:

  #### Slide 1: Model Checking and Formal Control Synthesis

    - Briefly explain what we mean by model checking and formal control synthesis.

    - At the same time add some text about model checking and formal control synthesis (even the definitions if needed)

    - Add some good visuals for model checking and formal control synthesis


  #### Slide 2: Temporal Logic

    - Add an animation of a car trying to perform perpendicular parking in an ongoing traffic and it is waiting for the cars to clear out so that it can park.

    - In the same animation you can show how the abstract real life requirement of "Park without hitting any obstacles" can be translated into a linear temporal logic specification.









## Slide Group 2: Background - Automata vs Temporal Logic Trees
  ### Script:
    
  #### Slide 1:    
    - Given our temporal logic specification we now have to proceed to solve our problems. A typical approach in the literature is to leverage the theory automata.
    Where one essentially creates an automaton out of the temporal logic specification and at the same time uses some reachability analysis tool on the dynamics. And then combines these two to find if any paths from the reachability analysis satisfy the temporal logic automaton.

    Although this process is well developed, it still falls SHORT when it comes to infinite systems. That is due to some inherent restrictions they have.

    - Explain some of the issues with the theory of automata for infinite systems, and how temporal logic trees deal with these problems.
      - Curse of dimensionality
      - Usually do not handle full LTL language
      - Usually lack online scalability

    - Explain how temporal logic trees utilize the reachability problem, and how they fascilitate a multi-reachability problem.

  #### Slide 2:
    - Now for those not very familiar with the idea of temporal logic trees, I will walk you through the intuition behind a tree formed by the parking specification that was defined earlier.

    - Now I claimed earlier that TLTs have some properties that are appealing to the infinite system problem. Typically reachability analysis algorithms do not scale very well with the complexity of the system, its dimension, and its length. From the structure of the tree we see that it abstracts away the complexity of the overall system from the reachability analysis, by breaking it down into the computation of reachabale and invariant sets for smaller problems, and then puts them all together.

    \TODO{SAY SOMETHING ABOUT THE LENGTH} 

  ### Slide Contents:
    
  #### Slide 1: 
    - Typically in the literature the theory of automata is used.

    - State some issues of automata when it comes to infinite systems, and how TLT deals with them.
    
  #### Slide 2:
    - Given the linear temporal logic specification you defined in the last slide, form the equivalent temporal logic tree.









## Slide Group 3: Zonotopes, Level sets, ...
  ### Script:
    - Clearly reachability analysis is still at the heart of the problem. There are multiple approaches on solving this problem, and the choice usually boils down to accuracy vs efficiency tradeoffs of the way we choose to represent the sets. There are lots and lots of options. Some of the most commonly used include level sets, polytopes, zonotopes and its variations. 

    - Here I provide a short list of some representation methods alongside some of their properties which are of great interest for us.

    - Even though ...
    
    - The natural structure of the temporal logic tree demands that we use a method that can deal with the unions, intersection, and overall disjoint sets, if we want to deal with meaningful problems and fully utilize the strengths of the trees.



  ### Slide Contents:
    #### Slide 1:
      - Add the table with the properties of some of the set representation methods.

    #### Slide 2:
      - Add plots of different kind of set representations e.g.,
        - Polytope
        - Zonotopes
        - Constrained Zonotopes
        - Hybrid Zonotopes

    - Add the mathematical expression of those sets (G-rep, H-rep, ...)




## Slide Group 4: Formally Introduce our test case
  ### Script:
    - Now to show you an example...

    - Describe the setup, that is two different lanes, cars can not cross lanes, and they are restricted to follow a counter-clockwise or clockwise direction.

    - What is the goal? What is the temporal logic specification we wanna analyize. What are the dynamics of the model (Just a point moving around? or...?)

  ### Slide Contents:
    - Show an example











## Slide Group 5: Summarize contributions
  ### Script:
    - Zonotopes are usually used for simple problems.

  ### Slide Contents:

## Slide Group 6: Conclusions and Future directions:
  ### Script:
    - Recap what the solution we currently have is.
    - Clarify which parts are not yet included in our solution.
    - Clarify that at this moment the thesis is not yet dealing with the formal control synthesis part.

  ### Slide Contents:
    -

## Optional:
  - Show an example with a simple zonotope and then with the hybrid zonotope, to show the importance of using hybrid zonotopes,
  and to further solidify the complexity of the problems current solutions might be dealing with.
































<!--

# Presentation:

## Slide: Temporal logic tree motivation:
  - Inherent restrictions of automaton-based methods for infinite systems (gao2021temporal)
    - Abstraction from infinite systems to finite systems suffers from the curse of dimensionality.
    - There are fewwe results for handling general LTL.
    - Current methods usually lack online scalability.
    - The controller obtained from automaton-based methods usually only contains a single control policy.

## Slide: Why zonotopes, and specifically Hybrid Zonotopes:


## Why combine those two instead of using only one of them? How do they complement each other?

-->













<!--

## Future directions:

### Implement the actual model of the car
  - Use the nonlinear bicycle model and then use the approach of liren's work for linearization while he was using constrained zonotopes
  - Split the state space into multiple regions and create a piecewise affine model.
  

### Perform a more thorough literature review to then make an accurate quantitative comparison between our approach and the state of the art.


-->




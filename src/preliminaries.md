Preliminaries:
    - Notation (DONE)
        - Write down the notation for the mathematics used in this paper. (DONE)

    - Plant Model (DONE)
        - Define the dynamic's model of a general nonlinear system. (DONE)

    - Reachability Analysis
        - For the visualization of the FRS and BRS, simply design a straight road and color the road ahead for the FRS and the road behind for the BRS.
        - For the visualization of the RCI, design a road that splits in two and add an obstacle in one of them. Show that assuming the car can not stop.
        - FRS
            - Definition
            - Visual Example
        - BRS
            - Definition
            - Visual Example
        - RCI
            - Definition
            - Visual Example

    - Linear Temporal Logic (DONE)
        - Definition        (DONE)
        - Syntax            (DONE)
        - Semantics         (DONE)

    - Temporal Logic Tree
        - Introduction (Need to be improved)
        - Definition (DONE)
        - Construction of TLT
            - The construction of the CTLT involves three steps
                - Step 1: Derive an equivalent LTL formula for $\varphi$ in weak-until positive normal form $\varphi_{N}$.
                    - In essence, to obtain the weak-until positive normal form one needs to get rid of all negations that are not directly applied to an atomic proposition, and make use of the formulas in \ref{chapter_preliminaries:eq:convert_ltl} to get rid of any \textit{Eventually} and \textit{Always} operators.
                - Step 2: Construct a CTLT for each atomic proposition.
                    - For each atomic proposition $p, \lnot p \in \mathcal{AP}$, its CTLT is a single set node. This set node contains all the states which satisfy $p$ or $\lnot$ accordingly. As an immediate result, the CTLT for $true$ and $false$ are the set nodes containing all the states and none of the states in the state space respectively.
                - Step 3: Inductively work up the tree to construct the CTLT for all subformulas of $\varphi_{N}$ and ultimately construct the CTLT for $\varphi_{N}$ itself.
                    - For the final step the CTLTs of interest to us are the ones corresponding to the operations $\land, \lor, \mathsf{U}, \mathsf{W}$.

                    - ($\varphi_{1} \land \varphi_{2}) and ($\varphi_{1} \lor \varphi_{2})
                        - The CTLTs for the $\land$ and $\lor$ operations are shown in Figure~\ref{}. The intuition it is that the root of the $\land$ tree represents all the states for which there exists a controlled trajectory that satisfies both $\varphi_{1}$ and $\varphi_{2}$, hence the intersection of the two sets. Similarly, the root of the $\lor$ tree represents all the states for which there exists a controlled trajectory that satisfies either $\varphi_{1}$ or $\varphi_{2}$, hence the union of the two sets.

                    - $\varphi_{1} \mathsf{U} \varphi_{2}$
                        - The CTLT for the $\mathsf{U}$ operation is shown in Figure~\ref{}. To reason with its structure, one can observe how the set of states for which there exists a controlled trajectory that satisfies $\varphi_{1} until it satisfies \varphi_{2}$ is equivalent to finding all the states which satisfy $\varphi_{1}$ and at the same are in the backwards reachable set with target the set of states satisfying $\varphi_{2}$. As an immediate result, a state which could satisfy such a formula is one which satisfies the aforementioned condition (left-hand side in Figure~\ref{}), or a state which is already in the set of states satisfying $\varphi_{2}$ (right-hand side in Figure~\ref{}).

                    - $\varphi_{1} \mathsf{W} \varphi_{2}$
                        - The CTLT for the $\mathsf{W}$ operation is shown in Figure~\ref{}. Although, the formula is in the weak-until positive normal form, to get an intuitive understanding of the semantics of the $\mathsf{W}$ operation, one can think of it as a combination of $\mathsf{U} and $\square$. That is, $\varphi_{1} \mathsf{W} \varphi_{2} = \varphi_{1} \mathsf{U} \varphi_{2} \lor \square \varphi_{1}$.

                        - Following this idea the equivalence between the right-hand side of the CTLT for $\mathsf{W}$ with the CTLT for $\mathsf{U}$ is justified. For the left-hand side of the CTLT in Figure~\ref{}, one can reason with it through the always operator being equivalent to some extend to the \text{Always} temporal operator.

                    Although this is not a formal proof, it provides an intuitive understanding of the CTLT construction. For a more formal proof of the construction of the CTLT, the reader is referred to \todo{[gao2021temporal]}.

        - Example

    - Set Representation
        - As it became evident in the previous section, the construction of the CTLT requires the ability to compute the intersection, union and backwards reachable set of sets. In this section we provide an overview of different set representations as well as motivate the use of hybrid zonotopes in this work.

        - Starting with the ability of different set representation tools to compute exact or approximate sets for some of the fundamental operations in set theory we provide an overview in Table~\ref{}.

            - In this table include the following set representation tools:
                - Polytope
                - Zonotope
                - Constrained Zonotope
                - Hybrid Zonotope
                - Level Set

            - In this table include the following operations
                - Linear Transformation
                - Minkowski Sum
                - Minkowski Difference
                - ...
                - Intersection
                - Union
                - Complement
                
            - Table caption: If an exact form of the operation is available, it is denoted by $E$, otherwise with $U$ and/or $O$ for under-approximation and over-approximation respectively. - denotes that we do not have any information about the ability of the tool to compute the operation. 

        Aside from the tool's ability to compute approximations or the exact form of the operations, another important aspect is the ability to deal with non-convex and disjoint sets. In Table~\ref{} we provide an overview of the ability of different set representation tools to deal with non-convex and disjoint sets.

            - In this table include the following set representation tools:
                - Polytope
                - Zonotope
                - Constrained Zonotope
                - Hybrid Zonotope
                - Level Set

            - Table caption: If the set representation tool can deal with non-convex and disjoint sets, it is denoted by $\cmark$, otherwise with $\xmark$.

        Out of all the representation tools mentioned, the ones that can realistically be used to handle the complexity of constructing CTLTs are the level sets and the hybrid zonotopes. However, as it was already mentioned in the introduction, level sets suffer from high computational complexity, and are therefore not preferred for online computations. On the other hand, hybrid zonotopes can handle the full complexity of constructing CTLTs but at the same time it partially retains the computational speed advantages of zonotopes. Therefore, in this work we will be using hybrid zonotopes as the set representation tool of choice.

    - Hybrid Zonotope

        - The hybrid zonotope is an extension of the zonotop and constrained zonotope, capable of handling non-convex and disjoint sets. Hence, this section starts with a brief overview of the zonotope and constrained zonotope, followed by the definition of the hybrid zonotope.


        - Definitions
            - Definition of Zonotope
            - Definition of Constrained Zonotope
            - Definition of Hybrid Zonotope
            - Provide a visual example of all three side by side

        - Set Operations 
            - Linear Transformation
            - Minkowski Sum
            - Minkowski Difference
                - Provide the definition using intersections and...
            - Intersection
            - Union
            - Complement
            - BRS
            - RCI 
                - Provide the formula for RCI expressed as BRS and intersections
            - Point containment
            - Empty set

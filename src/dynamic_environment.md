Dynamic Environment
    - Now it is time to move the dynamic environment case...

    - Before we proceed to the full scale definition of the dynamic environment, we will first confine the number of moving cars to one. This will allow us to focus on how to handle dynamic obstacles and then we will extend the definition to multiple moving cars. Assume now the simplified environment shown in Figure~\ref{}. There we see how the red car is approaching the intersection from left, which suggests that if the ego-vehicle would like to cross the intersection from above, it could result in a collision between the two vehicles. As an example of where the ego-vehicle could begin its motion, we added two more cars (blue, grey) approaching the intersection from the top to show the interaction between the possible starting points of the ego-vehicle and the moving obstacle.

    \figure{}

    - So we are now called upon to define an LTL formula that allows us to study wether an ego-vehicle can safely cross the intersection. One idea, as was done in [\todo{jiang2020ensuring}] to avoid static obstacles is to define an LTL as
    
    $\varphi = \square \lnot o,$
    $o = \{ \mathcal{O} \}$...


    The resulting safe space for this formula is highlighted by BLUE/GREEN in Figure~\ref{a}. Clearly this definition is not sufficient to guarantee to avoid collisions with moving obstacles as the obstacle is moving, as it is evident by Figure~\ref{} that the ... car would actually colide with the red car. 
    
    \figure

    Another idea is to replace the static obstacle set $\mathcal{O}, with the dynamic obstacle set $\mathcal{O}_d$ which is defined as the set of all possible obstacle trajectories. This, however, could generally and even more so in this case, result in a conservative solution as it would demand that the ego-vehicle would not start at any point that the obstacle could reach. This is illustrated in Figure~\ref{} where the ... car is supposed to be in a non-safe region as its safety can not be guaranteed, even though we see can see it safely cross the intersection in Figure~\ref{}.

    \figure 

    Therefore, we now introduce the idea of incorporating time information when computing the unsafe regions introduced due to the motion of the obstacle. To do so, we predict the trajectory of the moving obstacle by computing its forward reachable set and then for each time step $k$ in its prediction horizon, we compute the regions that would inevitably collide with the k-step forward reachable set in k steps from now.

    
    
    Explain the LTL formula to be used here...

    The TLT generated from this formula and the set of state satisfying it are shown in Figure~\ref{} and Figure~\ref{} respectively. We can see that in this case the formula correctly identifies that the ... car will not safely cross the intersection, while the ... car will.

    \figure (TLT)
    \figure (Safe space)

    The examples we just layed out showcase how our approach can be used to compute guaranteed safe region due to the presence of moving obstacles. However, we have not yet defined the formal definition of the dynamic environment. We do so now.


    \subsection{Proof...}
    Given...
# Slide 2:
    - Motivate for formal methods.

# Slide 3 - Introduction - Temp. Logic and Reachability Analysis:
    - Introduce the idea of using temporal logic and/or reachability analysis as an example for formal methods.
    - This slide is intended more for the audience that are not well familiar with temporal logic and/or reachability analysis,
    so that they can understand the contents before the background section.

# Slide 4 - Automata vs TLT:
    - Motivate use of TLTs over Automata.

# Slide 5 - Goals & Contributions:
    - Provide a general outline of our process.
    - List our main contributions.

# Slide 6 - Background - Reachability and Invariance:
    - Introduce the concept of reachable and invariant sets

# Slide 7 - Background - Reachability and Invariance - Example:
    - Provide examples for each set to ensure that everyone has at least an intuitive understanding of these concepts.

# Slide 8 - Background - Temporal Logic:
    - More formally introduce the idea of LTL and TLTs

# Slide 9 - Background - Temporal Logic - Example:
    - Provide an example to glue reachability with TLTs together.
    - Use the static environment case study to prepare them for the following sections.

# Slide 10 - Background - Set Representation:
    - Motivate the choice of Hybrid Zonotopes

# Slide 11 - Background - Hybrid Zonotope:
    - Define Hybrid Zonotope
    - Provide an example to ensure that everyone has a feeling of what the Hybrid Zonotope is.

# Slide 12 - Methodology - Static Environment:
    - In this case study I just want to slowly walk the audience through the process of generating the TLT and what each set node represents.
    - This case study is used as an inbetween step before we proceed to explain how we handle dynamic obstacles.

# Slide 13 - Methodology - Dynamic Environment:
    - Introduce a minimal example to explain our approach without confusing the audience with all five cars going around.

# Slides 14, 15 - Methodology - Dynamic Environment - First/Second Attempt:
    - These slides will be used only to provide some alternative ideas which fail. That is because I want to make the audience to understand the difference, 
    and why we even need to go through all this effort to compute the safe sets in such a way.

# Slide 16 - Methodology - Dynamic Environment - Third Attempt:
    - This is our approach where I will explain how we are now predicting where the obstacle could end up and work our way backwards from there to find the safe regions.
    - This slide will most likely be changed cause I feel I might need to add figures for each step of the process explaining the different reachable sets being computed.

# Slide 17 - Results - Static Envirnoment:
    - (I will replace the image with the videos later to add other colors as well if needed)
    - In this case study I wanna show the ability of Hybrid Zonotopes to handle non-convex disjoint sets.

# Slide 18 - Results - Dynamic Environment:
    - (I will replace the image with the videos later to add other colors as well if needed)
    - In this case study I wanna show how we are dealing with moving obstacles on a larger scale.
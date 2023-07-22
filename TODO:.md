# Next:
- There seems to be an issue with the computation of the forward reachable sets whenever the oa_cz_to_hypercube_tight_v2 method is used
- There seems to be an issue with the computation of the intersection of hybrid zonotopes.


SOS!!!
FIX IN ALL THE is_inside_... and is_empty methods the mistake you just fixed in the is_empty_hz




- Given a Zonotope 'Z'

- Define the over-approximating zonotope 'O(Z)'

- Compute its complement 'O(C(Z))'

- Compute the intersection of 'Z' with 'O(C(Z))'

- Check if the intersection is an empty set

- Keep on reducing the values of the generators until the intersection is a non-empty set














- Redesign the environment:
    - Make it square
        - This is so that we can perhaps simplify the representation of the Hybrid zonotopes for the inner and outer lanes.

- Check 'Convex Enclosures' from Trevor's PhD






- Check for over-approximations of HZ






- Check what is going wrong with the forward and backward reachable sets.
















TODO:

- Pay rent
- Finish with the over-approximation of constrained zonotopes
- Call bank for the 18-25 card (check what time the bank opens)
DONE    - Book flight tickets for Stockholm
- Ask Miguel about his Thesis defense
- Read papers for Kyriako's idea

- Alternative idea for over-approximation
    - Find vertices of cz (vertex enumeration)
    - Compute the centroid of the entire convex polytope using the vertices
    - Find midpoint between all consecutive vertices
    - Sort the midpoints according to their distance from the centroid (min to max)
    
    
    - Discretize the space every n degrees (e.g., 45 degrees)
    - Assign midpoints to the closest direction of the discretized space based on the angle the vector centroid->midpoint forms
    
    
    
    - In each direction that has available midpoints select the one with the lowest distance.
    - These midpoints are the generators of the zonotope.
    - Check if all vertices are included in the formed zonotope.
    - If they are included the over-approximation is done.
    - If they are not included, then proceed to add one more midpoint in each direction that still has available midpoints (the next smallest in distance)
    - Repeat this process until all the vertices are included in the formed zonotope.

    - To check if a vertex is contained in the zonotope, adapt the linear program in scott's work to be used for zonotopes.
    Simply ignore the constraints


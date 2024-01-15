# RTSEngine
Simple real-time-strategy game engine. Does Path-finding of groups of agents on a dynamic constrained Delaunay triangulation. Agents can have various sizes but the forces for bigger agents are not very "good-looking" yet. Can build buildings and walls with edges poinitng in one of 8 general directions (N,S,W,E,NE,NW,SE,SW). Technically, this is a Starcraft 2 clone.

Works on LINUX. Will work on Windows once I finish everything.


# Constrained Delaunay Triangulation (CDT)
is an algorithm for creating triangulated meshes with edges between specified vertices. Implementation is not yet very optimized and is based closely on: https://doi.org/10.1016/0141-1195(87)90043-X . 
So far we can add buildings very fast, but removal is still slow because the entire mesh is reconstructed from scratch.

# Reduced Triangulation 
is a graph created from the CDT where the mesh is abstracted into "crossroads" and "corridords" (and "dead ends" but they are not important really...). 

 - "Crossroad" is a triangle which has 3 uncostrained (= not being next to a wall) edges. 
 - "Corridor" is a collection of connected triangles each having exactly 2 unconstrained edges connecting 2 "Crossroads"

# PathFinding - A* 

Pathfinding then finds a path represented by connected triangles from start to end triangle
by doing **A*** only on "crossroads". (This significantly reduces the number of triangles that A* needs to go through)


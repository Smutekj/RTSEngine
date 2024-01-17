# RTSEngine
Simple real-time-strategy game engine. Does Path-finding of groups of agents on a dynamic constrained Delaunay triangulation. Agents can have various sizes but the forces for bigger agents are not very "good-looking" yet. Can build buildings and walls with edges poinitng in one of 8 general directions (N,S,W,E,NE,NW,SE,SW). Technically, this is a Starcraft 2 clone.

Works on LINUX. Will work on Windows once I finish everything.


# Constrained Delaunay Triangulation (CDT)
is an algorithm for creating triangulated meshes with edges between specified vertices. Implementation is not yet very optimized and is based closely on: https://doi.org/10.1016/0141-1195(87)90043-X . 
So far we can add buildings very fast, but removal is still slow because the entire mesh is reconstructed from scratch. If it turns out to be too hard to remove dynamically buildings without constructing them, I will just make the building destruction animations slow and dramatic so that there is enough time to create new mesh. Another advantage of ignoring fast removal is the fact that I can put more data in a triangle 

# Reduced Triangulation 
is a graph created from the CDT where the mesh is abstracted into "crossroads" and "corridords" (and "dead ends" but they are not important really...). 

 - "Crossroad" is a triangle which has 3 uncostrained (= not being next to a wall) edges. 
 - "Corridor" is a collection of connected triangles each having exactly 2 unconstrained edges connecting 2 "Crossroads"

# PathFinding - A* 

Pathfinding is done in three steps:

 1.  A collection of connected triangles from start to end triangle is found using A* on the Reduced triangulation graph. Union of these triangles is a single polygon containing the desired path (called Funnel).
 2.  A Funnel is extracted from the triangle-path
 3.  The actual path (a bunch of straight lines) is obtained from Funnel via the Funnel algorithm (nice explanation here: http://ahamnett.blogspot.com/2012/10/funnel-algorithm.html)
 4.  ???
 5.  Profit

By doing **A*** only on "crossroads" we significantly reduce the number of triangles that A* needs to go through.

# Demonstration: Single agent pathfinding:
Purple polygon is the funnel path is light blue and portals are black.

![](https://github.com/Smutekj/RTSEngine/blob/main/PathFinding-Single.gif)

# Demonstration: Multiple agents pathfinding with fog of war:

![](https://github.com/Smutekj/RTSEngine/blob/main/PathFinding-Groups.gif)

# Agents Equations of Motion: Boids

 The Equations of motions governing the agents is closely based on the concept of Boids by Reynolds. (original article: https://www.cs.toronto.edu/~dt/siggraph97-course/cwr87/ , nice explanation: https://cs.stanford.edu/people/eroberts/courses/soco/projects/2008-09/modeling-natural-systems/boids.html )

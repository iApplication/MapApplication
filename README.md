# Object-Oriented-Design-for-a-Map-Application-based-on-Graph-Theory
Object Oriented Design for a Map Application based on Graph Theory

## Packet roadgraph
path: src/roadgraph

Class Name: MapGraph
This is the main class for my Graph data structure. In my design, MapGraph is a directed graph. MapGraph structure has the two classes MapNode and MapEdge as inner classes. I have chosen to use inner classes because accessing MapNode or MapEdge directly without going through MapGraph doesn't make sense. The MapNode class represents a node on a MapGraph data structure and the MapEdge class represents an edge between two MapNode's. 

Class Name: MapEdge
As described above, MapEdge is the link between two MapNode's, say, two road intersections in the road graph model. This class represents an edge and has properties such as from, to, road name, road type, length, etc.

Class Name: MapNode
The MapNode class represents a Node in MapGraph. Nodes have relationships with each other through the MapEdge class. It has properties such as geometric location of the node, the neighbor nodes of the node, and the edge linked to the node.

### Search Algorithm
#### BFS
In the BFS algorithm, I use queue structure to keep track of the node to be searched next. More details can be seen here: https://en.wikipedia.org/wiki/Breadth-first_search

Limitation of BFS: BFS only accountS number of edges, not distance
#### Dijkstra and A*
Dijkstra and A* use priority queue to keep track of next node to be explored. A* also add heuristic estimate cost which makes the algorithm converge faster.



## Acknowledgement
The starter code is from UCSD Mooc team.

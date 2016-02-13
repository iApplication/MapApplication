/**
 * @author Jun Lu
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import geography.GeographicPoint;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.ListIterator;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import util.GraphLoader;

public class MapGraph {
	//XXX: member variables
	private int numVertices;
	private int numEdges;
	private Map<GeographicPoint,MapNode> vertices;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// XXX: Implement in this constructor
		numVertices = 0;
		numEdges = 0;
		vertices = new HashMap<GeographicPoint,MapNode>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		return numVertices;
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		return vertices.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		return numEdges;
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// XXX: Implement this method
		if(vertices.containsKey(location) || null==location){
			return false;
		}
		vertices.put(location, new MapNode(location));
		List<MapNode> neighbors = new ArrayList<MapNode>();
		vertices.get(location).setNeighbours(neighbors);
		//adjListsMap.put(location,  neighbors);
		numVertices ++;
		return true;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

		//XXX: Implement this method
		if(!vertices.containsKey(from) || !vertices.containsKey(to)) {
			throw new IllegalArgumentException("MapGraph do not contain the vertex!");
		}
		else if(null==roadName || null==roadType || null==from || null==to) {
			throw new IllegalArgumentException("Arguments cannot be null!");
		}
		else if(length < 0) {
			throw new IllegalArgumentException("edge length cannot be less than 0");
		}
		
		MapNode fromNode = vertices.get(from);
		MapNode toNode = vertices.get(to);
		
		MapEdge edge = new MapEdge(fromNode, toNode, roadName, roadType, length);
		vertices.get(from).getNeighbours().add(vertices.get(to));
		vertices.get(from).getAdjacentEdges().add(edge);
		numEdges ++;
		
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// XXX: Implement this method 
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());

		if (start == null || goal == null) {
			System.out.println("Start or goal node is null!  No path exists.");
			return null;
		}
		
		MapNode startNode = vertices.get(start);
		MapNode goalNode = vertices.get(goal);
		
		HashMap<MapNode, MapNode> parentMap = new HashMap<MapNode, MapNode>();
		boolean found = bfsSearch(startNode, goalNode, parentMap, nodeSearched);

		if (!found) {
			System.out.println("No path exists");
			return null;
		}
		// reconstruct the path
		List<MapNode> pathNode = constructPath(startNode, goalNode, parentMap);
		return getGeographyFromNode(pathNode);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param parentMap can help reconstruct the path, it contains the parent node of each node in the path.
	 * @return true if find the goal, false otherwise
	 */
	private boolean bfsSearch(MapNode start, MapNode goal, 
			HashMap<MapNode, MapNode> parentMap, Consumer<GeographicPoint> nodeSearched) {
		HashSet<MapNode> visited = new HashSet<MapNode>();
		Queue<MapNode> toExplore = new LinkedList<MapNode>();
		
		toExplore.add(start);
		boolean found = false;
		while (!toExplore.isEmpty()) {
			MapNode curr = toExplore.remove();
			nodeSearched.accept(curr.getLocation()); // hook function
			if (curr.equals(goal)) {
				found = true;
				break;
			}
			List<MapNode> neighbors = curr.getNeighbours();
			ListIterator<MapNode> it = neighbors.listIterator(neighbors.size());
			while (it.hasPrevious()) {
				MapNode next = it.previous();
				if (!visited.contains(next)) {
					visited.add(next);
					parentMap.put(next, curr);
					toExplore.add(next);
				}
			}
		}
		
		return found;
		
	}
	
	/**
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param parentMap can help reconstruct the path, it contains the parent node of each node in the path.
	 * @return the path from start to goal.
	 */
	private List<MapNode> constructPath(MapNode start, MapNode goal, HashMap<MapNode, MapNode> parentMap) {
		LinkedList<MapNode> path = new LinkedList<MapNode>();
		MapNode curr = goal;
		while (curr != start) {
			path.addFirst(curr);
			curr = parentMap.get(curr);
		}
		path.addFirst(start);
		return path;
	}
	
	/** 
	 * 
	 * @param a list of MapNode
	 * @return the list of GeographicPoint corresponding to MapNode.
	 */
	private List<GeographicPoint> getGeographyFromNode(List<MapNode> nodes) {
		List<GeographicPoint> geographies = new ArrayList<GeographicPoint>();
		for(MapNode node : nodes) {
			geographies.add(node.getLocation());
		}
		
		return geographies;
	}
	

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// XXX: Implement this method

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		if (start == null || goal == null)
			throw new NullPointerException("Cannot find route from or to null node");
		
		MapNode startNode = vertices.get(start);
		MapNode goalNode = vertices.get(goal);
		if (startNode == null) {
			System.err.println("Start node " + start + " does not exist");
			return null;
		}
		if (goalNode == null) {
			System.err.println("End node " + goal + " does not exist");
			return null;
		}
		
		HashMap<MapNode,MapNode> parentMap = new HashMap<MapNode,MapNode>(); //used for reconstruct path
		// set of node to be explored, and the MapNode should implement comparable interface
		// and remember it is a min Priority Queue 
		PriorityQueue<MapNode> toExplore = new PriorityQueue<MapNode>(); 
		HashSet<MapNode> visited = new HashSet<MapNode>(); // used for marking the node 
		
		for (MapNode n : vertices.values()) {
			n.setDistance(Double.POSITIVE_INFINITY);
		}
		startNode.setDistance(0.0); // set the distance of the start Node to be 0 
		
		toExplore.add(startNode);
		boolean found = false;
		int count = 0;
		while(!toExplore.isEmpty()) {
			MapNode curr = toExplore.remove();
			count ++;
			nodeSearched.accept(curr.getLocation()); // hook function
			if (curr.equals(goalNode)) {
				found = true;
				break;
			}
			
			if(!visited.contains(curr)) {
				visited.add(curr);
				
				for(MapEdge edge : curr.getAdjacentEdges()) {
					MapNode next = edge.getTo();
					if(!visited.contains(next)) {
						Double nextDist = curr.getDistance() + edge.getLength();
						if(nextDist < next.getDistance()) {
							parentMap.put(next, curr);
							next.setDistance(nextDist);
							toExplore.add(next);
						}
					}
				}
			}
		}
		
		if (!found) {
			System.out.println("No path found from " +start+ " to " + goal);
			List<GeographicPoint> list = new ArrayList<GeographicPoint>();
			list.add(start);
			return list;
		}
		
		System.out.println("Find the goal in step of " + count);
		// reconstruct the path
		List<MapNode> pathNode = constructPath(startNode, goalNode, parentMap);
		return getGeographyFromNode(pathNode);
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// XXX: Implement this
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());

		if (start == null || goal == null)
			throw new NullPointerException("Cannot find route from or to null node");
		
		MapNode startNode = vertices.get(start);
		MapNode goalNode = vertices.get(goal);
		if (startNode == null) {
			System.err.println("Start node " + start + " does not exist");
			return null;
		}
		if (goalNode == null) {
			System.err.println("End node " + goal + " does not exist");
			return null;
		}
		
		HashMap<MapNode,MapNode> parentMap = new HashMap<MapNode,MapNode>(); //used for reconstruct path
		// set of node to be explored, and the MapNode should implement comparable interface
		// and remember it is a min Priority Queue 
		PriorityQueue<MapNode> toExplore = new PriorityQueue<MapNode>(); 
		HashSet<MapNode> visited = new HashSet<MapNode>(); // used for marking the node 
		
		for (MapNode n : vertices.values()) {
			n.setDistance(Double.POSITIVE_INFINITY);
			n.setPredDistance(Double.POSITIVE_INFINITY);
		}
		startNode.setDistance(0.0); // set the distance of the start Node to be 0 
		
		toExplore.add(startNode);
		boolean found = false;
		int count = 0;
		while(!toExplore.isEmpty()) {
			MapNode curr = toExplore.remove();
			count ++;
			nodeSearched.accept(curr.getLocation()); // hook function
			if (curr.equals(goalNode)) {
				found = true;
				break;
			}
			
			if(!visited.contains(curr)) {
				visited.add(curr);
				
				for(MapEdge edge : curr.getAdjacentEdges()) {
					MapNode next = edge.getTo();
					if(!visited.contains(next)) {
//						Double nextDist = curr.getDistance() + edge.getLength();
//						if(nextDist < next.getDistance()) {
//							parentMap.put(next, curr);
//							next.setDistance(nextDist);
//							toExplore.add(next);
//						}
						// different from dijkstra as following:
						double nextDist = curr.getDistance() + edge.getLength();
						// core of A* is just to add to currDist the cost of getting to
						// the destination
						double predDist = nextDist+ (next.getLocation()).distance(goalNode.getLocation());
						if(predDist < next.getPredDistance()){
							parentMap.put(next, curr);
							next.setDistance(nextDist);
							next.setPredDistance(predDist);
							toExplore.add(next);
						}
					}
				}
			}
		}
		
		if (!found) {
			System.out.println("No path found from " +start+ " to " + goal);
			List<GeographicPoint> list = new ArrayList<GeographicPoint>();
			list.add(start);
			return list;
		}
		
		System.out.println("Find the goal in step of " + count);
		// reconstruct the path
		List<MapNode> pathNode = constructPath(startNode, goalNode, parentMap);
		return getGeographyFromNode(pathNode);

		
	}

	
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		System.out.println("DONE.");
		
		// You can use this method for testing.  
		
		/* Use this code in Week 3 End of Week Quiz
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/
		
	}
	
}

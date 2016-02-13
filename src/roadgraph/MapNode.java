/**
 * @author Jun Lu
 * 
 * Node class contains the location, neighbors and edge information concerning 
 * to each node in the graph.
 */

package roadgraph;

import geography.GeographicPoint;

import java.util.ArrayList;
import java.util.List;

public class MapNode {
	
	private GeographicPoint location;	//Represents geographical location of the vertex
	private List<MapNode> neighbours;	//Represents the list of neighbours of the vertex
	private List<MapEdge>	adjacentEdges;	//Represents the list of adjacent edges from the vertex
	
	public MapNode(GeographicPoint location) {
		this.location = location;
		this.adjacentEdges = new ArrayList<MapEdge>();
		this.neighbours = new ArrayList<MapNode>();
	}
	
	@Override
	public boolean equals(Object o) {
		if (this == o) return true;
	    if (o == null || getClass() != o.getClass()) return false;

	    MapNode node = (MapNode) o;

	    if (location != null ? !location.equals(node.location) : node.location != null) 
	    	return false;
	    if (neighbours != null ? !neighbours.equals(node.neighbours) : node.neighbours != null) 
	    	return false;
	    return adjacentEdges != null ? adjacentEdges.equals(node.adjacentEdges) : node.adjacentEdges == null;

	}

	public GeographicPoint getLocation() {
		return location;
	}

	public void setLocation(GeographicPoint location) {
		this.location = location;
	}

	public List<MapNode> getNeighbours() {
		return neighbours;
	}

	public void setNeighbours(List<MapNode> neighbours) {
		this.neighbours = neighbours;
	}

	public List<MapEdge> getAdjacentEdges() {
		return adjacentEdges;
	}

	public void setAdjacentEdges(List<MapEdge> adjacentEdges) {
		this.adjacentEdges = adjacentEdges;
	}
	
	

}

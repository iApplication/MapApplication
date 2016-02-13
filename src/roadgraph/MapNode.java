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

public class MapNode implements Comparable{
	
	private GeographicPoint location;	//Represents geographical location of the vertex
	private List<MapNode> neighbours;	//Represents the list of neighbours of the vertex
	private List<MapEdge>	adjacentEdges;	//Represents the list of adjacent edges from the vertex
	private Double distance; // Represents the actual distance from a specific start node
	private Double predDistance; // Represents the predicted distance from a specific start node
	
	public MapNode(GeographicPoint location) {
		this.location = location;
		this.adjacentEdges = new ArrayList<MapEdge>();
		this.neighbours = new ArrayList<MapNode>();
	}
	
	@Override
	public boolean equals(Object o) {
		if (!(o instanceof MapNode) || (o == null)) {
			return false;
		}
		MapNode node = (MapNode)o;
		return node.location.equals(this.location);

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

	public Double getDistance() {
		return distance;
	}

	public void setDistance(Double distance) {
		this.distance = distance;
	}

	@Override
	public int compareTo(Object o) {
		// convert to map node, may throw exception
		MapNode m = (MapNode)o; 
		return ((Double)this.getDistance()).compareTo((Double) m.getDistance());
	}

	public Double getPredDistance() {
		return predDistance;
	}

	public void setPredDistance(Double predDistance) {
		this.predDistance = predDistance;
	}
	
	
	

}

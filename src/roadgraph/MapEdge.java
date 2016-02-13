/**
 * @author Jun Lu
 * 
 * Edge class is used to store all attributes of any single edge in one item. 
 * These attributes are name, type, length, starting and end point.
 *
 */

package roadgraph;

import geography.GeographicPoint;

public class MapEdge {
	private MapNode from;
	private MapNode to;
	private String roadName;
	private String roadType;
	private double length;
	
	public MapEdge(MapNode from, MapNode to, String roadName,
			String roadType, double length)
	{
		this.from = from;
		this.to = to;
		this.roadName = roadName;
		this.roadType = roadType;
		this.length = length;
	}

	public MapNode getFrom() {
		return from;
	}

	public void setFrom(MapNode from) {
		this.from = from;
	}

	public MapNode getTo() {
		return to;
	}

	public void setTo(MapNode to) {
		this.to = to;
	}

	public String getRoadName() {
		return roadName;
	}

	public void setRoadName(String roadName) {
		this.roadName = roadName;
	}

	public String getRoadType() {
		return roadType;
	}

	public void setRoadType(String roadType) {
		this.roadType = roadType;
	}

	public double getLength() {
		return length;
	}

	public void setLength(double length) {
		this.length = length;
	}

}

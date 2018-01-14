package graphDefinition;


import java.util.ArrayList;
import java.util.List;

public class Vertex<VState> {

	private VState state;
	private List<Vertex<VState>> adjacentVertices;
	private String uniqueId;
	private boolean isExplored;

	public Vertex(VState state, String uniqueId) {
		super();
		this.state = state;
		adjacentVertices = new ArrayList<Vertex<VState>>();
		this.uniqueId = uniqueId;
		isExplored= false;
	}

	public boolean isExplored() {
		return isExplored;
	}

	public void setExplored(boolean isExplored) {
		this.isExplored = isExplored;
	}

	public List<Vertex<VState>> getAdjacentVertices() {
		return adjacentVertices;
	}

	public String getUniqueId() {
		return uniqueId;
	}

	public VState getState() {
		return state;
	}


}

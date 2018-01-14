package graphDefinition;

public class Edge<EState> {

	private EState edgeState;
	private Vertex<?> firstEndVertex;
	private Vertex<?> secondEndVertex;

	public Edge(EState edgeState, Vertex<?> firstEndVertex, Vertex<?> secondEndVertex) {
		super();
		this.edgeState = edgeState;
		this.firstEndVertex = firstEndVertex;
		this.secondEndVertex = secondEndVertex;
	}

	public Vertex<?> getFirstEndVertex() {
		return firstEndVertex;
	}

	public Vertex<?> getSecondEndVertex() {
		return secondEndVertex;
	}

	public EState getEdgeState() {
		return edgeState;
	}


}
package graphDefinition;


import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;

public class Graph<VState, EState> {

    private List<Vertex<VState>> vertexList;
    private List<Edge<EState>> edgeList;
    private HashMap<String, Vertex<VState>> verIdMap;
    private HashMap<HashSet<Vertex<VState>>, Edge<EState>> edgeMap;
    

    public Graph() {
      
        this.vertexList = new ArrayList<>();
        this.verIdMap = new HashMap<>();
        this.edgeMap = new HashMap<>();
        this.edgeList = new ArrayList<>();
    }

    public void addVertex(VState vertexState, String uniqueID) {
        Vertex<VState> newVer = new Vertex<>(vertexState, uniqueID);
        verIdMap.put(uniqueID, newVer);
        vertexList.add(newVer);
    }

    public void addEdge(EState edgeState, Vertex<VState> a, Vertex<VState> b) {
        HashSet<Vertex<VState>> set = new HashSet<>();
        set.add(a);
        set.add(b);
        Edge<EState> edge = new Edge<>(edgeState, a, b);
        edgeMap.put(set, edge);
        edgeList.add(edge);
        a.getAdjacentVertices().add(b);
        b.getAdjacentVertices().add(a);
    }

    public Vertex<VState> getVertex(String id) {
        return verIdMap.get(id);

    }

    public List<Vertex<VState>> getVertexList() {
        return vertexList;
    }

    public HashMap<String, Vertex<VState>> getVerIdMap() {
        return verIdMap;
    }

    @SuppressWarnings("unchecked")
    public void removeEdge(Edge<EState> anEdge) {
        edgeList.remove(anEdge);
        HashSet<Vertex<VState>> set = new HashSet<>();
        set.add((Vertex<VState>) anEdge.getFirstEndVertex());
        set.add((Vertex<VState>) anEdge.getSecondEndVertex());
        edgeMap.remove(set);
        anEdge.getFirstEndVertex().getAdjacentVertices().remove(anEdge.getSecondEndVertex());
        anEdge.getSecondEndVertex().getAdjacentVertices().remove(anEdge.getFirstEndVertex());

    }

    public List<Edge<EState>> getEdgeList() {
        return edgeList;
    }

    public HashMap<HashSet<Vertex<VState>>, Edge<EState>> getEdgeMap() {
        return edgeMap;
    }

    public int getSize() {
        return this.vertexList.size();
    }

  
}


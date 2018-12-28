package misc.graphs;

import datastructures.concrete.ArrayDisjointSet;
import datastructures.concrete.ArrayHeap;
import datastructures.concrete.ChainedHashSet;
import datastructures.concrete.DoubleLinkedList;
import datastructures.concrete.KVPair;
import datastructures.concrete.dictionaries.ChainedHashDictionary;
import datastructures.interfaces.IDictionary;
import datastructures.interfaces.IDisjointSet;
import datastructures.interfaces.IList;
import datastructures.interfaces.IPriorityQueue;
import datastructures.interfaces.ISet;
import misc.exceptions.NoPathExistsException;

/**
 * Represents an undirected, weighted graph, possibly containing self-loops, parallel edges,
 * and unconnected components.
 *
 * Note: This class is not meant to be a full-featured way of representing a graph.
 * We stick with supporting just a few, core set of operations needed for the
 * remainder of the project.
 */
public class Graph<V, E extends Edge<V> & Comparable<E>> {
    // NOTE 1:
    //
    // Feel free to add as many fields, private helper methods, and private
    // inner classes as you want.
    //
    // And of course, as always, you may also use any of the data structures
    // and algorithms we've implemented so far.
    //
    // Note: If you plan on adding a new class, please be sure to make it a private
    // static inner class contained within this file. Our testing infrastructure
    // works by copying specific files from your project to ours, and if you
    // add new files, they won't be copied and your code will not compile.
    //
    //
    // NOTE 2:
    //
    // You may notice that the generic types of Graph are a little bit more
    // complicated then usual.
    //
    // This class uses two generic parameters: V and E.
    //
    // - 'V' is the type of the vertices in the graph. The vertices can be
    //   any type the client wants -- there are no restrictions.
    //
    // - 'E' is the type of the edges in the graph. We've contrained Graph
    //   so that E *must* always be an instance of Edge<V> AND Comparable<E>.
    //
    //   What this means is that if you have an object of type E, you can use
    //   any of the methods from both the Edge interface and from the Comparable
    //   interface
    //
    // If you have any additional questions about generics, or run into issues while
    // working with them, please ask ASAP either on Piazza or during office hours.
    //
    // Working with generics is really not the focus of this class, so if you
    // get stuck, let us know we'll try and help you get unstuck as best as we can.
    
    private E[][] adjacencyMatrix;
    private IDictionary<V, Integer> vertexIds;
    private IDictionary<Integer, V> vertexElements;
    
    private int numVertices;
    private int numEdges;

    /**
     * Constructs a new graph based on the given vertices and edges.
     *
     * @throws IllegalArgumentException  if any of the edges have a negative weight
     * @throws IllegalArgumentException  if one of the edges connects to a vertex not
     *                                   present in the 'vertices' list
     */
    public Graph(IList<V> vertices, IList<E> edges) {        
        this.numVertices = vertices.size();
        this.numEdges = edges.size();
        
        this.adjacencyMatrix = make2dArrayOfE(numVertices);
        this.vertexIds = new ChainedHashDictionary<V, Integer>();
        this.vertexElements = new ChainedHashDictionary<Integer, V>();
        
        int index = 0;
        for (E edge : edges) {
            if (edge.getWeight() < 0) {
                throw new IllegalArgumentException();
            }
            if (!vertices.contains(edge.getVertex1()) || !vertices.contains(edge.getVertex2())) {
                throw new IllegalArgumentException();
            }
            
            // insert vertex into dictionaries
            V vertex1 = edge.getVertex1();
            V vertex2 = edge.getVertex2();
            if (!vertexIds.containsKey(vertex1)) {
                vertexIds.put(vertex1, index);
                vertexElements.put(index, vertex1);
                index++;
            }
            if (!vertexIds.containsKey(vertex2)) {
                vertexIds.put(vertex2, index);
                vertexElements.put(index, vertex2);
                index++;
            }
            
            // insert edge into adjacency matrix
            int vertex1Id = vertexIds.get(vertex1);
            int vertex2Id = vertexIds.get(vertex2);
            //System.out.println(adjacencyMatrix[vertex1Id][vertex2Id] == null);
            if (adjacencyMatrix[vertex1Id][vertex2Id] == null || 
                    (adjacencyMatrix[vertex1Id][vertex2Id] != null && 
                    edge.compareTo(adjacencyMatrix[vertex1Id][vertex2Id]) < 0)) {
                adjacencyMatrix[vertex1Id][vertex2Id] = edge;
                adjacencyMatrix[vertex2Id][vertex1Id] = edge;
            }
        }
        
    }
    
    @SuppressWarnings("unchecked")
    private E[][] make2dArrayOfE(int size) {
        return (E[][]) new Edge[size][size];
    }

    /**
     * Sometimes, we store vertices and edges as sets instead of lists, so we
     * provide this extra constructor to make converting between the two more
     * convenient.
     */
    public Graph(ISet<V> vertices, ISet<E> edges) {
        // You do not need to modify this method.
        this(setToList(vertices), setToList(edges));
    }

    // You shouldn't need to call this helper method -- it only needs to be used
    // in the constructor above.
    private static <T> IList<T> setToList(ISet<T> set) {
        IList<T> output = new DoubleLinkedList<>();
        for (T item : set) {
            output.add(item);
        }
        return output;
    }

    /**
     * Returns the number of vertices contained within this graph.
     */
    public int numVertices() {
        return numVertices;
    }

    /**
     * Returns the number of edges contained within this graph.
     */
    public int numEdges() {
        return numEdges;
    }

    /**
     * Returns the set of all edges that make up the minimum spanning tree of
     * this graph.
     *
     * If there exists multiple valid MSTs, return any one of them.
     *
     * Precondition: the graph does not contain any unconnected components.
     */
    public ISet<E> findMinimumSpanningTree() {
        IDisjointSet<V> vertexDisjointSet = new ArrayDisjointSet<V>();
        IPriorityQueue<E> graphEdges = new ArrayHeap<E>();
        ISet<E> mstEdges = new ChainedHashSet<E>();
        
        for (int i = 0; i < numVertices; i++) {
            vertexDisjointSet.makeSet(vertexElements.get(i));

            for (int j = 0; j < numVertices; j++) {
                if (adjacencyMatrix[i][j] != null) {
                    graphEdges.insert(adjacencyMatrix[i][j]);
                }
            }
        }
        
        while (!graphEdges.isEmpty() && mstEdges.size() < this.numVertices - 1) {
            E edge = graphEdges.removeMin();
            
            V uSetRepVertex = edge.getVertex1();
            V vSetRepVertex = edge.getVertex2();
            
            if (vertexDisjointSet.findSet(uSetRepVertex) != vertexDisjointSet.findSet(vSetRepVertex)) {
                mstEdges.add(edge);
                vertexDisjointSet.union(uSetRepVertex, vSetRepVertex);
            }
        }
        
        return mstEdges;
    }

    /**
     * Returns the edges that make up the shortest path from the start
     * to the end.
     *
     * The first edge in the output list should be the edge leading out
     * of the starting node; the last edge in the output list should be
     * the edge connecting to the end node.
     *
     * Return an empty list if the start and end vertices are the same.
     *
     * @throws NoPathExistsException  if there does not exist a path from the start to the end
     */
    public IList<E> findShortestPathBetween(V start, V end) {
        IPriorityQueue<VertexWrapper<V>> minDistanceVertices = new ArrayHeap<>();
        IDictionary<V, Double> vertexDistances = new ChainedHashDictionary<>();
        IDictionary<V, Boolean> verticesProcessed = new ChainedHashDictionary<>();
        IDictionary<V, E> vertexPredecessorEdges = new ChainedHashDictionary<>();
        IList<E> shortestPath = new DoubleLinkedList<>();
        
        // return empty list if start is end
        if (start.equals(end)) {
            return shortestPath;
        }
        
        // set initial values for all vertices
        for (KVPair<Integer, V> vertexPair : vertexElements) {
            V vertex = vertexPair.getValue();
            if (vertex.equals(start)) {
                vertexDistances.put(vertex, 0.0);
            } else {
                vertexDistances.put(vertex, Double.POSITIVE_INFINITY);
            }
            verticesProcessed.put(vertex, false);
        }
        minDistanceVertices.insert(new VertexWrapper<V>(start, vertexDistances.get(start)));
        
        // iterate through all vertices
        while (!minDistanceVertices.isEmpty()) {
            // take out vertex with smallest distance and process it
            VertexWrapper<V> currVertexWrapper = minDistanceVertices.removeMin();
            V currVertex = currVertexWrapper.getVertex();
            int currVertexId = vertexIds.get(currVertex);
            verticesProcessed.put(currVertex, true);
            
            // iterate through vertice's neighbors
            for (int i = 0; i < numVertices; i++) {
                E currEdge = adjacencyMatrix[currVertexId][i];
                
                // update distance of neighbors with shorter distance
                if (currEdge != null) {
                    V otherVertex = currEdge.getOtherVertex(currVertex);
                    double otherVertexNewDistance = vertexDistances.get(currVertex) + currEdge.getWeight();
                    if (otherVertexNewDistance < vertexDistances.get(otherVertex)) {
                        vertexDistances.put(otherVertex, otherVertexNewDistance);
                        vertexPredecessorEdges.put(otherVertex, currEdge);
                        minDistanceVertices.insert(new VertexWrapper<V>(otherVertex, otherVertexNewDistance));
                    }
                }
            }

            // remove all duplicates from priority queue
            while (!minDistanceVertices.isEmpty() && verticesProcessed.get(minDistanceVertices.peekMin().getVertex())) {
                minDistanceVertices.removeMin();
            }
        }
        
        // add all predecessor edges starting from end into final list
        V currVertex = end;
        while (!currVertex.equals(start)) {
            if (!vertexPredecessorEdges.containsKey(currVertex)) {
                throw new NoPathExistsException();
            }
            
            E currEdge = vertexPredecessorEdges.get(currVertex);
            shortestPath.insert(0, currEdge);
            currVertex = currEdge.getOtherVertex(currVertex);
        }
        
        // return list
        return shortestPath;
    }
    
    private static class VertexWrapper<V> implements Comparable<VertexWrapper<V>> {
        private double distance;
        private V vertex;
        
        public VertexWrapper(V vertex, double distance) {
            this.vertex = vertex;
            this.distance = distance;
        }
        
        public V getVertex() {
            return this.vertex;
        }
        
        public Double getDistance() {
            return this.distance;
        }
        
        @Override
        public int compareTo(VertexWrapper<V> other) {
            return Double.compare(distance, other.getDistance());
        }
    }
    
}
 
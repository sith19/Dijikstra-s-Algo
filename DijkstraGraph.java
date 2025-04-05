// === CS400 File Header Information ===
// Name: Yogev Rachbuch
// Email: rachbuch@wisc.edu
// Group and Team: <your group name: two letters, and team color>
// Group TA: <name of your group's ta>
// Lecturer: Florian Heimerl
// Notes to Grader: <optional extra notes>
import org.junit.jupiter.api.Test;
import java.util.PriorityQueue;
import java.util.List;
import java.util.ArrayList;
import java.util.NoSuchElementException;
import org.junit.jupiter.api.Assertions;
/**
 * This class extends the BaseGraph data structure with additional methods for
 * computing the total cost and list of node data along the shortest path
 * connecting a provided starting to ending nodes. This class makes use of
 * Dijkstra's shortest path algorithm.
 */
public class DijkstraGraph<NodeType, EdgeType extends Number>
        extends BaseGraph<NodeType, EdgeType>
        implements GraphADT<NodeType, EdgeType> {

    /**
     * While searching for the shortest path between two nodes, a SearchNode
     * contains data about one specific path between the start node and another
     * node in the graph. The final node in this path is stored in its node
     * field. The total cost of this path is stored in its cost field. And the
     * predecessor SearchNode within this path is referened by the predecessor
     * field (this field is null within the SearchNode containing the starting
     * node in its node field).
     *
     * SearchNodes are Comparable and are sorted by cost so that the lowest cost
     * SearchNode has the highest priority within a java.util.PriorityQueue.
     */
    protected class SearchNode implements Comparable<SearchNode> {
        public Node node;
        public double cost;
        public SearchNode predecessor;

        public SearchNode(Node node, double cost, SearchNode predecessor) {
            this.node = node;
            this.cost = cost;
            this.predecessor = predecessor;
        }

        public int compareTo(SearchNode other) {
            if (cost > other.cost)
                return +1;
            if (cost < other.cost)
                return -1;
            return 0;
        }
    }

    /**
     * Constructor that sets the map that the graph uses.
     */
    public DijkstraGraph() {
        super(new PlaceholderMap<>());
    }

    /**
     * This helper method creates a network of SearchNodes while computing the
     * shortest path between the provided start and end locations. The
     * SearchNode that is returned by this method is represents the end of the
     * shortest path that is found: it's cost is the cost of that shortest path,
     * and the nodes linked together through predecessor references represent
     * all of the nodes along that shortest path (ordered from end to start).
     *
     * @param start the data item in the starting node for the path
     * @param end   the data item in the destination node for the path
     * @return SearchNode for the final end node within the shortest path
     * @throws NoSuchElementException when no path from start to end is found
     *                                or when either start or end data do not
     *                                correspond to a graph node
     */
    protected SearchNode computeShortestPath(NodeType start, NodeType end) {
        Node starting;
        Node ending;
        //Initialize starting and end node variables
        
        try {
          starting = this.nodes.get(start);
          ending = this.nodes.get(end);
          //get nodes from map
        }catch (NoSuchElementException e) {
          throw new NoSuchElementException("Start, End Or Both Are Not In The Graph");
          //if either of the nodes are nonexistent throw a NoSuchElementException
        }
        
        SearchNode ret = null;
        //create a SearchNode to store the return value
        
        BaseGraph< SearchNode, Double> endGraph = new BaseGraph<SearchNode, Double>(new PlaceholderMap<>());
        //create a graph to store the network of SearchNodeds
        
        PlaceholderMap<Node, Node> visited = new PlaceholderMap<Node, Node>();
        //make a map to store what nodes have been visited
        
        PriorityQueue<SearchNode> pq = new PriorityQueue<SearchNode>();
        //Initialize a priority queue for Dijkstra's algorithm
        
        pq.add(new SearchNode(starting, 0, null));
        //add the starting node to the priority queue  
  
        while(!pq.isEmpty()) {
          
          SearchNode dest = pq.remove();
          //get the next shortest edge
          if(!visited.containsKey(dest.node)) {
            //check if the end of the edge has not been visited
            
            if(dest.node.equals(starting) && dest.predecessor == null && dest.cost == 0) {
              //check if dest is equal to the initial node inserted to the priority queue
              visited.put(dest.node, dest.node);
              //mark it as visited
              endGraph.insertNode(dest);
              //add it to the graph but don't create an edge to avoid a NullPointerException
            }else {
              visited.put(dest.node, dest.node);
              //mark as visited
              endGraph.insertNode(dest);
              //add node to the graph
              endGraph.insertEdge(dest.predecessor, dest, dest.cost);
              //create edge between node and predecessor
            }
            
            if(dest.node.equals(ending)){
              ret = dest;
              // if the destination is equal to the end node, store it for return
            }
            List<Edge> edgesLeaving = dest.node.edgesLeaving;
            //get all the edges leaving the destination node
            for(Edge e: edgesLeaving) {
              //iterate over every edge e in the list
              if(!visited.containsKey(e.successor)) {
                //check if destination node of edge was visited
                pq.add(new SearchNode(e.successor, dest.cost + Double.valueOf(e.data.toString()) , dest));
                //if its unvisited insert into the priority queue a new search nodes with the neighbor, destination and cost + weight of edge
              }
            }
            
          }
        }
        if(ret == null) {
          //if ret is still null it means there is no path between start and end 
          throw new NoSuchElementException("Path Does Not Exist Between " + start + " to " + end);
          //throw a new exception
        }
        return ret;
        //otherwise return a reference to the ending node
    }

    /**
     * Returns the list of data values from nodes along the shortest path
     * from the node with the provided start value through the node with the
     * provided end value. This list of data values starts with the start
     * value, ends with the end value, and contains intermediary values in the
     * order they are encountered while traversing this shorteset path. This
     * method uses Dijkstra's shortest path algorithm to find this solution.
     *
     * @param start the data item in the starting node for the path
     * @param end   the data item in the destination node for the path
     * @return list of data item from node along this shortest path
     */
    public List<NodeType> shortestPathData(NodeType start, NodeType end) {
        
        SearchNode endNode = this.computeShortestPath(start, end);
        //get reference to end node from computeShortestPath
        List<NodeType> pathData = new ArrayList<NodeType>();
        //Initiate a new list using array list
        
        while(endNode != null) {
          pathData.add(0, endNode.node.data);
          //add the current endnode to the start of the list
          endNode = endNode.predecessor;
          //set end node to its predecessor
        }
        return pathData;
        //return the finished list an exception will be thrown if computeShortestPath throws one
	}

    /**
     * Returns the cost of the path (sum over edge weights) of the shortest
     * path freom the node containing the start data to the node containing the
     * end data. This method uses Dijkstra's shortest path algorithm to find
     * this solution.
     *
     * @param start the data item in the starting node for the path
     * @param end   the data item in the destination node for the path
     * @return the cost of the shortest path between these nodes
     */
    public double shortestPathCost(NodeType start, NodeType end) {
        SearchNode endNode = this.computeShortestPath(start, end);
        //get end node from computeShortestPath
        return endNode.cost;
        //use endNode's cost field 
    }

    // TODO: implement 3+ tests in step 4.1
    /**
     * Tests Florian's lecture example with the costs and paths calculated during lecture
     */
    @Test
    public void testLectureExample() {
      DijkstraGraph<Character, Integer> graph = new DijkstraGraph<Character, Integer>();
      graph.insertNode('A');
      graph.insertNode('B');
      graph.insertNode('C');
      graph.insertNode('D');
      graph.insertNode('E');
      graph.insertNode('F');
      graph.insertNode('G');
      graph.insertNode('H');
      graph.insertEdge('A', 'B', 4);
      graph.insertEdge('A', 'C', 2);
      graph.insertEdge('B', 'D', 1);
      graph.insertEdge('B', 'E', 10);
      graph.insertEdge('C', 'D', 5);
      graph.insertEdge('D', 'E', 3);
      graph.insertEdge('D', 'F', 0);
      graph.insertEdge('F', 'D', 2);
      graph.insertEdge('F', 'H', 4);
      graph.insertEdge('G', 'H', 4);
      //initializes graph from florian's lecture notes
      
      Assertions.assertEquals(0, graph.shortestPathCost('A', 'A'));
      Assertions.assertEquals(4, graph.shortestPathCost('A', 'B'));
      Assertions.assertEquals(2, graph.shortestPathCost('A', 'C'));
      Assertions.assertEquals(5, graph.shortestPathCost('A', 'D'));
      Assertions.assertEquals(8, graph.shortestPathCost('A', 'E'));
      Assertions.assertEquals(5, graph.shortestPathCost('A', 'F'));
      Assertions.assertEquals(5, graph.shortestPathCost('A', 'F'));
      Assertions.assertEquals(9, graph.shortestPathCost('A', 'H'));
      //checks path weights calculated during lecture with A as starting nodes
      
      List<Character> nodelist = graph.shortestPathData('A', 'E');
      List<Character> comp = List.of('A','B','D','E');
      //get path data and create a list with the answer from lecture
      if(!nodelist.equals(comp)) {
        Assertions.fail("Shortest Path Doesn't Return Expected List Between A and E");
        //compare the lists and if they are not equal fail
      }
    
      nodelist = graph.shortestPathData('A', 'F');
      comp = List.of('A','B','D','F');
      //get path data and create a list with the answer from lecture
      if(!nodelist.equals(comp)) {
        Assertions.fail("Shortest Path Doesn't Return Expected List Between A and F");
        //compare the lists and if they are not equal fail
      }
      
    }
    /**
     * tests graph from lecture example but with different start and end nodes
     */
    @Test
    public void testLectureExampleVariation() {
      DijkstraGraph<Character, Integer> graph = new DijkstraGraph<Character, Integer>();
      //insert nodes
      graph.insertNode('A');
      graph.insertNode('B');
      graph.insertNode('C');
      graph.insertNode('D');
      graph.insertNode('E');
      graph.insertNode('F');
      graph.insertNode('G');
      graph.insertNode('H');
      
      //insert edges
      graph.insertEdge('A', 'B', 4);
      graph.insertEdge('A', 'C', 2);
      graph.insertEdge('B', 'D', 1);
      graph.insertEdge('B', 'E', 10);
      graph.insertEdge('C', 'D', 5);
      graph.insertEdge('D', 'E', 3);
      graph.insertEdge('D', 'F', 0);
      graph.insertEdge('F', 'D', 2);
      graph.insertEdge('F', 'H', 4);
      graph.insertEdge('G', 'H', 4);
      
      Assertions.assertEquals(1, graph.shortestPathCost('B', 'D'));
      Assertions.assertEquals(4, graph.shortestPathCost('B', 'E'));
      Assertions.assertEquals(1, graph.shortestPathCost('B', 'F'));
      Assertions.assertEquals(5, graph.shortestPathCost('B', 'H'));
      //tests path weights with B as the starting node
      
      Assertions.assertEquals(2, graph.shortestPathCost('F', 'D'));
      Assertions.assertEquals(5, graph.shortestPathCost('F', 'E'));
      Assertions.assertEquals(4, graph.shortestPathCost('F', 'H'));
      //tests path weights with F as the starting node
      
      Assertions.assertEquals(5, graph.shortestPathCost('C', 'D'));
      Assertions.assertEquals(8, graph.shortestPathCost('C', 'E'));
      Assertions.assertEquals(5, graph.shortestPathCost('C', 'F'));
      Assertions.assertEquals(9, graph.shortestPathCost('C', 'H'));
      //tests path weights with C as the starting node
      
      
      List<Character> nodelist = graph.shortestPathData('C', 'H');
      List<Character> comp = List.of('C','D','F','H');
      //get path data and create a list with the correct path data
      
      if(!nodelist.equals(comp)) {
        Assertions.fail("Shortest Path Doesn't Return Expected List Between C and H");
        //compare the lists and if they are not equal fail
      }
      
      
      nodelist = graph.shortestPathData('F', 'E');
      comp = List.of('F','D','E');
      //get path data and create a list with the correct path data
      if(!nodelist.equals(comp)) {
        Assertions.fail("Shortest Path Doesn't Return Expected List Between F and E");
        //compare the lists and if they are not 
      }
      
      nodelist = graph.shortestPathData('B', 'E');
      comp = List.of('B','D','E');
      //get path data and create a list with the correct path data
      if(!nodelist.equals(comp)) {
        Assertions.fail("Shortest Path Doesn't Return Expected List Between B and E");
        //compare the lists and if they are not 
      }
      
      nodelist = graph.shortestPathData('B', 'F');
      comp = List.of('B','D','F');
      //get path data and create a list with the correct path data
      if(!nodelist.equals(comp)) {
        Assertions.fail("Shortest Path Doesn't Return Expected List Between B and F");
        //compare the lists and if they are not 
      }
    }
    /**
     * Tests that the methods throw a NoSuchElementException when a non existent path is provided
     */
    @Test
    public void testNonExistantPaths() {
      DijkstraGraph<Character, Integer> graph = new DijkstraGraph<Character, Integer>();
      // Insert nodes
      graph.insertNode('X');
      graph.insertNode('Y');
      graph.insertNode('Z');
      graph.insertNode('W');
      graph.insertNode('V');
      graph.insertNode('U');
      graph.insertNode('T');

      // Insert edges with weights
      graph.insertEdge('X', 'Y', 3);
      graph.insertEdge('X', 'Z', 7);
      graph.insertEdge('Y', 'W', 2);
      graph.insertEdge('Z', 'W', 1);
      graph.insertEdge('W', 'V', 5);
      graph.insertEdge('V', 'U', 4);
      graph.insertEdge('U', 'T', 6);
      graph.insertEdge('V', 'Z', 2);
      
      Assertions.assertThrows(NoSuchElementException.class, () -> graph.computeShortestPath('T', 'U'));
      
      Assertions.assertThrows(NoSuchElementException.class, () -> graph.computeShortestPath('Z', 'X'));
      
      Assertions.assertThrows(NoSuchElementException.class, () -> graph.computeShortestPath('U', 'Z'));
      //tests that nonexistent paths throw NoSuchElementException for computeShortestPath
      
      Assertions.assertThrows(NoSuchElementException.class, () -> graph.shortestPathData('T', 'U'));
      
      Assertions.assertThrows(NoSuchElementException.class, () -> graph.shortestPathData('Z', 'X'));
      
      Assertions.assertThrows(NoSuchElementException.class, () -> graph.shortestPathData('U', 'Z'));
      //checks that the exception propagates for PathData method with nonexistent path
      
      
      Assertions.assertThrows(NoSuchElementException.class, () -> graph.shortestPathCost('T', 'U'));
      
      Assertions.assertThrows(NoSuchElementException.class, () -> graph.shortestPathCost('Z', 'X'));
      
      Assertions.assertThrows(NoSuchElementException.class, () -> graph.shortestPathCost('U', 'Z'));
      //checks that the exception propagates for PathCost method with nonexistent path
     
    }
    /**
     * Tests extra configurations with a different graph and that the methods 
     * throw exceptions for non existent node configs
     */
    @Test
    public void testExtraConfigirations() {
      DijkstraGraph<Character, Integer> graph = new DijkstraGraph<Character, Integer>();
      // Insert nodes
      graph.insertNode('X');
      graph.insertNode('Y');
      graph.insertNode('Z');
      graph.insertNode('W');
      graph.insertNode('V');
      graph.insertNode('U');
      graph.insertNode('T');

      // Insert edges with weights
      graph.insertEdge('X', 'Y', 3);
      graph.insertEdge('X', 'Z', 7);
      graph.insertEdge('Y', 'W', 2);
      graph.insertEdge('Z', 'W', 1);
      graph.insertEdge('W', 'V', 5);
      graph.insertEdge('V', 'U', 4);
      graph.insertEdge('U', 'T', 6);
      graph.insertEdge('V', 'Z', 2);
      
      Assertions.assertEquals(2, graph.shortestPathCost('Y', 'W'));
      Assertions.assertEquals(3, graph.shortestPathCost('V', 'W'));
      Assertions.assertEquals(10, graph.shortestPathCost('Z', 'U'));
      Assertions.assertEquals(14, graph.shortestPathCost('X', 'U'));
     
      
      List<Character> nodelist = graph.shortestPathData('Y', 'W');
      List<Character> comp = List.of('Y','W');
      //get path data and create a list with the correct path data
      
      if(!nodelist.equals(comp)) {
        Assertions.fail("Shortest Path Doesn't Return Expected List Between Y and W");
        //compare the lists and if they are not equal fail
      }
      
      
      nodelist = graph.shortestPathData('V', 'W');
      comp = List.of('V','Z','W');
      //get path data and create a list with the correct path data
      if(!nodelist.equals(comp)) {
        Assertions.fail("Shortest Path Doesn't Return Expected List Between V and W");
        //compare the lists and if they are not 
      }
      
      nodelist = graph.shortestPathData('Z', 'U');
      comp = List.of('Z','W','V', 'U');
      //get path data and create a list with the correct path data
      if(!nodelist.equals(comp)) {
        Assertions.fail("Shortest Path Doesn't Return Expected List Between Z and U");
        //compare the lists and if they are not 
      }
      
      nodelist = graph.shortestPathData('X', 'U');
      comp = List.of('X','Y','W', 'V', 'U');
      //get path data and create a list with the correct path data
      if(!nodelist.equals(comp)) {
        Assertions.fail("Shortest Path Doesn't Return Expected List Between B and F");
        //compare the lists and if they are not 
      }
      
      Assertions.assertThrows(NoSuchElementException.class, () -> graph.computeShortestPath('A', 'U'));
      
      Assertions.assertThrows(NoSuchElementException.class, () -> graph.computeShortestPath('Z', 'A'));
      
      Assertions.assertThrows(NoSuchElementException.class, () -> graph.computeShortestPath('A', 'B'));
      //tests that nonexistent nodes throw NoSuchElementException for computeShortestPath
      
      Assertions.assertThrows(NoSuchElementException.class, () -> graph.shortestPathData('A', 'U'));
      
      Assertions.assertThrows(NoSuchElementException.class, () -> graph.shortestPathData('Z', 'A'));
      
      Assertions.assertThrows(NoSuchElementException.class, () -> graph.shortestPathData('A', 'B'));
      //checks that the exception propagates for PathData method with nonexistent nodes
      
      
      Assertions.assertThrows(NoSuchElementException.class, () -> graph.shortestPathCost('A', 'U'));
      
      Assertions.assertThrows(NoSuchElementException.class, () -> graph.shortestPathCost('Z', 'A'));
      
      Assertions.assertThrows(NoSuchElementException.class, () -> graph.shortestPathCost('A', 'B'));
      //checks that the exception propagates for PathCost method with nonexistent nodes
    }
    
}

package projects.multipath.advanced;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.PrintWriter;
import java.io.Reader;
import java.io.Serializable;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.Map;
import java.util.Set;
import java.util.Vector;
import java.awt.geom.Point2D;
import javafx.util.Pair;
import com.google.gson.Gson;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;

public class Problem implements Serializable {
	private static String FILE_FOLDER = "D:\\temp\\data\\dmrpp\\"; 
	
	private static final long serialVersionUID = 1L;
	
	public Graph graph = null;
	public int sg[][] = null;
	
	public  Map<Integer, Integer> vidNewIdMap = null;
	public  Map<Integer, Integer> newIdVidMap = null;

	public Problem() {}
	
	public Problem(Graph graph, int[][] sg) {
		super();
		this.graph = graph;
		this.sg = sg;
	}
	
	
	/**
	 * Write the graph in plain text to a file. The file consists the follwoing lines
	 * 1. Vertices in the format vid:x:y, separated by space. Note that x, y may be empty
	 * 2. The set of edges in v1:v2 format, separated by space
	 * 3. The start & goal locations in the format svid:gvid, separated by space
	 * @param fileName
	 */
	private void writeToFile(String fileName){
		try{
			// Create directory structure if needed
			File file = new File(fileName);
			file.getParentFile().mkdirs();
			
			// Open printwriter 
			PrintWriter pw = new PrintWriter(file);
			StringBuffer sbuf = new StringBuffer(); 
			
			// Write out the vertices
			for(int i = 0; i <graph.vertices.length; i ++){
				if(i > 0)pw.print(" ");
				pw.print(graph.vertices[i].id + ":" + graph.vertices[i].getIX()+ ":" + graph.vertices[i].getIY());
				
				// Append edges to buffer
				Integer[] nbrs = graph.adjacencySetMap.get(graph.vertices[i].id).toArray(new Integer[0]);
				for(int j = 0; j < nbrs.length; j ++){
					if(sbuf.length() > 0)sbuf.append(" ");
					sbuf.append(graph.vertices[i].id+":"+nbrs[j]);
				}
			}
			pw.println();
			
			// Write the edges
			pw.println(sbuf);
			
			// Write the starts
			for(int i = 0; i < sg[0].length; i ++){
				if(i > 0){pw.print(" ");}
				pw.print(sg[0][i]+":"+sg[1][i]);
			}
			pw.flush();
			pw.close();
		}
		catch(IOException e){}
	}
	
	
	public static void writeToFile(Problem p, String fileName){
		ObjectOutputStream out;
		try {
			out = new ObjectOutputStream(new FileOutputStream(fileName));
			out.writeObject(p.graph);
			out.writeObject(p.sg);
			out.close();
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	public static Pair<Problem, Double> readFromJsonFile(String fileName){
		
		Problem p = new Problem();
		double percentObstacle = 0;
		Gson gson = new Gson();

		try {
			Reader reader = Files.newBufferedReader(Paths.get(fileName));
			JsonObject instance = gson.fromJson(reader, JsonObject.class);

			reader.close();

			JsonObject meta = instance.getAsJsonObject("meta");
			int number_of_robots = meta.getAsJsonPrimitive("number_of_robots").getAsInt();
			JsonArray shape = meta.getAsJsonObject("description")
								.getAsJsonObject("parameters")
								.getAsJsonArray("shape");
			int rows = shape.get(0).getAsInt();
			int cols = shape.get(1).getAsInt();

			Vector<Point2D> obstacles = new Vector<>();
			for (JsonElement obstacleElement : instance.getAsJsonArray("obstacles")) {
				JsonArray obstacleArray = obstacleElement.getAsJsonArray();
				Point2D obstaclePoint = new Point2D.Double(obstacleArray.get(0).getAsInt(), 
														   obstacleArray.get(1).getAsInt());
				obstacles.add(obstaclePoint);
			}
			
			p.graph = Graph.createGeneric2DGridGraphWithHoles(rows, cols, obstacles);
			percentObstacle = (double)obstacles.size() / (rows*cols);
			
			p.sg = new int[2][number_of_robots];
			JsonArray starts = instance.getAsJsonArray("starts");
			JsonArray targets = instance.getAsJsonArray("targets");
			for (int i = 0; i < number_of_robots; i++) {
				JsonArray startArray = starts.get(i).getAsJsonArray();
				p.sg[0][i] = p.graph.getId(startArray.get(0).getAsInt(), startArray.get(1).getAsInt());
				JsonArray targetArray = targets.get(i).getAsJsonArray();
				p.sg[1][i] = p.graph.getId(targetArray.get(0).getAsInt(), targetArray.get(1).getAsInt());
			}

			p.graph = Graph.convertGraph(p.graph, p.sg[0], p.sg[1]);
		}
		catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		// probably should've encapsulated the double inside p or p.graph, 
		// but this would require a lot of changes to keep consistency
		// or some deriving class  
		return new Pair<Problem, Double>(p, percentObstacle);
	}

	public static Problem readFromFile(String fileName){
		ObjectInputStream in;
		Problem p = new Problem();
		try {
			in = new ObjectInputStream(new FileInputStream(fileName));
			p.graph = (Graph)(in.readObject());
			p.sg = (int[][])(in.readObject());
			in.close();
		} catch (ClassNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return p;
	}
	
	/**
	 * Create a col x row grid problem with fracObs obstacles and n agents
	 * @param col
	 * @param row
	 * @param fracObs
	 * @param n
	 * @return
	 */
	public static Problem createGridProblem(int col, int row, double fracObs, int n){
		Problem p = new Problem();
		p.graph = Graph.createGeneric2DGridGraphWithHoles(col, row, fracObs);
		p.sg = Graph.getRandomStartGoal(p.graph, n);
		p.graph = Graph.convertGraph(p.graph, p.sg[0], p.sg[1]);
		return p;
	}

	/**
	 * Create a col x row grid problem with fracObs obstacles and n agents
	 * @param col
	 * @param row
	 * @param fracObs
	 * @param n
	 * @return
	 */
	public static Problem createGridProblem8Connected(int col, int row, double fracObs, int n){
		Problem p = new Problem();
		p.graph = Graph.createGeneric2DGridGraphWithHoles8Connected(col, row, fracObs);
		p.sg = Graph.getRandomStartGoal(p.graph, n);
		p.graph = Graph.convertGraph(p.graph, p.sg[0], p.sg[1]);
		return p;
	}

	/**
	 * Create a n^-puzzle
	 * @param n
	 * @return
	 */
	public static Problem createN2Puzzle(int n){
		Problem p = new Problem();
		p.graph = Graph.create2DGridGraph(n, n, true);
		p.sg = Graph.getRandomStartGoalMany(p.graph, n*n);
		return p;
	}
	
	/**
	 * Create a n^-puzzle
	 * @param n
	 * @return
	 */
	public static Problem createN2M1Puzzle(int n){
		Problem p = new Problem();
		p.graph = Graph.create2DGridGraph(n, n, true);
		p.sg = Graph.getRandomStartGoalMany(p.graph, n*n-1);
		return p;
	}
	
	public static void main(String argv[]){
		if(argv.length > 0) FILE_FOLDER = argv[0];
		// Allow the files to close properly
		try {
			createN2Puzzles();
			createCrowdedHardProblems();
			create32x32Problems();
			create32x32Problems8Connected();
			create24x18PerformanceTestingProblems();
			Thread.sleep(5000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	
	protected static void createN2Puzzles(){
		for(int i = 0; i < 100; i++){
			for(int n = 3; n < 6; n ++){
				Problem p = Problem.createN2Puzzle(n);
				p.writeToFile(FILE_FOLDER + "\\n2puzzle\\" + (n*n) + "-puzzle-" + (1001 + i) + ".txt");
				Problem.writeToFile(p, FILE_FOLDER + "\\n2puzzle\\" + (n*n) + "-puzzle-" + (1001 + i) + ".dat");
			}
		}
		
	}
	
	protected static void createCrowdedHardProblems(){
		for(int i = 0; i < 10; i ++){
			for(int a = 10; a <= 60; a = a + 10){
				Problem p = Problem.createGridProblem(8, 8, 0, a);
				p.writeToFile(FILE_FOLDER + "\\8x8-grid\\" + a + "-agts-" + (1001 + i) + ".txt");
				Problem.writeToFile(p, FILE_FOLDER + "\\8x8-grid\\" + a + "-agts-" + (1001 + i) + ".dat");
			}
		}
		for(int i = 0; i < 10; i ++){
			for(int a = 10; a <= 250; a = a + 10){
				Problem p = Problem.createGridProblem(16, 16, 0, a);
				p.writeToFile(FILE_FOLDER + "\\16x16-grid\\" + a + "-agts-" + (1001 + i) + ".txt");
				Problem.writeToFile(p, FILE_FOLDER + "\\16x16-grid\\" + a + "-agts-" + (1001 + i) + ".dat");
			}
		}
	}

	
	protected static void create32x32Problems(){
		for(int i = 0; i < 10; i ++){
			for(int n = 10; n <= 200; n=n+10){
				Problem p = Problem.createGridProblem(32, 32, 0.2, n);
				p.writeToFile(FILE_FOLDER + "\\32x32-grid\\20-pct-obs-" + n + "-agts-" + (1001 + i) + ".txt");
				Problem.writeToFile(p, FILE_FOLDER + "\\32x32-grid\\20-pct-obs-" + n + "-agts-" + (1001 + i) + ".dat");
			}
		}
	}

	protected static void create32x32Problems8Connected(){
		for(int i = 0; i < 10; i ++){
			for(int n = 10; n <= 400; n=n+10){
				Problem p = Problem.createGridProblem8Connected(32, 32, 0.2, n);
				p.writeToFile(FILE_FOLDER + "\\32x32-grid-8c\\20-pct-obs-" + n + "-agts-" + (1001 + i) + ".txt");
				Problem.writeToFile(p, FILE_FOLDER + "\\32x32-grid-8c\\20-pct-obs-" + n + "-agts-" + (1001 + i) + ".dat");
			}
		}
	}

	protected static void create24x18PerformanceTestingProblems(){
		for(int obs = 0; obs < 35; obs=obs+5){
			for(int n = 10; n <= 300; n=n+10){
				for(int i = 0; i < 10; i ++){
					Problem p = Problem.createGridProblem(24, 18, obs/100., n);
					p.writeToFile(FILE_FOLDER + "\\24x18-grid\\" + n + "-agt-"+ (obs) + "-pct-obs-" + (1001 + i) + ".txt");
					Problem.writeToFile(p, FILE_FOLDER + "\\24x18-grid\\" + n + "-agt-"+ (obs) + "-pct-obs-" + (1001 + i) + ".dat");
				}
			}
		}
	}
	
	
	
	
	
	protected static void createCrowdedHardProblems8Connected(){
		for(int i = 0; i < 10; i ++){
			for(int a = 10; a < 61; a = a + 10){
				Problem p = Problem.createGridProblem8Connected(8, 8, 0, a);
				Problem.writeToFile(p, FILE_FOLDER + "8x8-grid-8c\\" + a + "-agts-" + (1001 + i) + ".txt");
			}
		}
		for(int i = 0; i < 10; i ++){
			for(int a = 10; a < 251; a = a + 10){
				Problem p = Problem.createGridProblem8Connected(16, 16, 0, a);
				Problem.writeToFile(p, FILE_FOLDER + "16x16-grid-8c\\" + a + "-agts-" + (1001 + i) + ".txt");
			}
		}
	}

	protected static Problem getA9Puzzle(){
		Graph g = new Graph();
		g.addVertex(0, new int[]{0, 1, 3});
		g.addVertex(1, new int[]{0, 1, 2, 4});
		g.addVertex(2, new int[]{1, 2, 5});
		g.addVertex(3, new int[]{0, 3, 4, 6});
		g.addVertex(4, new int[]{1, 3, 4, 5, 7});
		g.addVertex(5, new int[]{2, 4, 5, 8});
		g.addVertex(6, new int[]{3, 6, 7});
		g.addVertex(7, new int[]{4, 6, 7, 8});
		g.addVertex(8, new int[]{5, 7, 8});
		g.finishBuildingGraph();
		int sg[][] = new int[][]{{3, 0, 2, 8, 1, 4, 7, 5, 6},{0, 1, 2, 3, 4, 5, 6, 7, 8}};
		
		Problem p = new Problem();
		p.graph = g;
		p.sg = sg;
		return p;
	}
	
	public static Problem getLongStraightWithOneGarageProblem(){
		Graph g = new Graph();
		g.addVertex(0, new int[]{0, 1});
		g.addVertex(1, new int[]{0, 1, 2});
		g.addVertex(2, new int[]{1, 2, 3});
		g.addVertex(3, new int[]{2, 3, 4});
		g.addVertex(4, new int[]{3, 4, 5, 9});
		g.addVertex(5, new int[]{4, 5, 6});
		g.addVertex(6, new int[]{5, 6, 7});
		g.addVertex(7, new int[]{6, 7, 8});
		g.addVertex(8, new int[]{7, 8});
		g.addVertex(9, new int[]{4, 9});
		g.finishBuildingGraph();
		int sg[][] = new int[][]{{0, 8},{8, 0}};
		
		Problem p = new Problem();
		p.graph = g;
		p.sg = sg;
		return p;
	}
}

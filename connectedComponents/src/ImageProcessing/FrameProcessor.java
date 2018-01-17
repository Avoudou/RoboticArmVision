package ImageProcessing;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.opencv.core.Mat;

import com.badlogic.gdx.math.Vector3;

import detection.FocusState;
import graphDefinition.Graph;
import graphDefinition.Vertex;
import graphSearch.ConnectedComponents;
import graphSearch.ImageMatrixCell;

public class FrameProcessor {

	private List<Coordinates> velocityEstimationList;
	private HashMap<FocusState, Coordinates> jointPosistions = new HashMap<>();
	private int zeroDistanceCompSize = 100;
	private int referanceDistanceCompSize = 1000;
	private int referanceDistance = 500;
	private final int STEPS_TO_GET_MEAN_VELOCITY = 6;

	public FrameProcessor(int zeroZsize, int referanceZsize) {
		velocityEstimationList = new ArrayList<>();
		this.zeroDistanceCompSize = zeroZsize;
		this.referanceDistance = referanceZsize;
	}

	public Coordinates getCoordinatesOfSignificantObject(Mat mat, FocusState objectDescription) {
		ConnectedComponents componentAnalyzer = new ConnectedComponents(mat);
		Graph<ImageMatrixCell, ?> objectGraph = componentAnalyzer.getBiggestComponent();
		if (objectGraph != null && objectGraph.getSize() > 0) {
			Coordinates cords = getMeanCenterOfGraph(objectGraph, objectDescription.toString());
			System.out.println("Coords for " + objectDescription.toString().toLowerCase() + ": X = " + cords.getX() + " Y = " + cords.getY()
					+ " Size :" + objectGraph.getSize());
			jointPosistions.put(objectDescription, cords);
	
			if (objectDescription == FocusState.BALL) {
				if (velocityEstimationList.size() < STEPS_TO_GET_MEAN_VELOCITY) {
					velocityEstimationList.add(cords);
				} else {
					velocityEstimationList.remove(0);
					velocityEstimationList.add(cords);
				}
			}

			return cords;
		}
		return new Coordinates(-999, -999, -999);
	}

	public Coordinates getMeanCenterOfGraph(Graph<ImageMatrixCell, ?> graph, String objectDescription) {
		int avgX = 0;
		int avgY = 0;
		int sumX = 0;
		int sumY = 0;
		for (Vertex<ImageMatrixCell> v : graph.getVertexList()) {
			sumX += v.getState().getX();
			sumY += v.getState().getY();
		}
		avgX = (int) Math.round(sumX / graph.getSize());
		avgY = (int) Math.round(sumY / graph.getSize());
		if (objectDescription == "ball") {
			return new Coordinates(avgX, 0, (int) calculateBallZ(graph));
		}
		return new Coordinates(avgX, avgY, 0);
	}

	public Coordinates getMovingObjectEstimatedCoordinates() {
		float pointAX = velocityEstimationList.get(velocityEstimationList.size() - 1).getX();
		float pointAY = velocityEstimationList.get(velocityEstimationList.size() - 1).getY();
		float pointAZ = velocityEstimationList.get(velocityEstimationList.size() - 1).getZ();
		Vector3 linePointA = new Vector3(pointAX, pointAY, pointAZ);

		float pointBX = velocityEstimationList.get(0).getX();
		float pointBY = velocityEstimationList.get(0).getY();
		float pointBZ = velocityEstimationList.get(0).getZ();
		Vector3 linePointB = new Vector3(pointAX, pointAY, pointAZ);

		Vector3 parralelToDirectionVector = new Vector3(linePointA);
		parralelToDirectionVector.sub(linePointB.cpy());

		Vector3 predictionVector = new Vector3();
		Vector3 predictedIntercectionPosition = linePointA.cpy().add(predictionVector.cpy());

		return new Coordinates((int) predictedIntercectionPosition.x, (int) predictedIntercectionPosition.y,
				(int) predictedIntercectionPosition.z);
	}

	private double calculateBallZ(Graph<ImageMatrixCell, ?> graph) {
		int currentCompSize = graph.getSize();
		double increment = (referanceDistanceCompSize - zeroDistanceCompSize) / referanceDistance * 1.0;
		double z = increment * (currentCompSize - zeroDistanceCompSize);
		return z;
	}

	public HashMap<FocusState, Coordinates> getJointPosistions() {
		return jointPosistions;
	}

}
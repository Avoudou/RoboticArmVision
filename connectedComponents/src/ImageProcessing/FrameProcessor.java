package ImageProcessing;

import java.util.LinkedList;

import org.opencv.core.Mat;

import com.badlogic.gdx.math.Vector3;

import detection.FocusState;
import detection.Launcher;
import graphDefinition.Graph;
import graphDefinition.Vertex;
import graphSearch.ConnectedComponents;
import graphSearch.ImageMatrixCell;

public class FrameProcessor {

	private LinkedList<Coordinates> velocityEstimationList = new LinkedList<>();
	private int zeroDistanceCompSize = 100;
	private int referanceDistanceCompSize = 1000;
	private int referanceDistance = 500;
	private final int STEPS_TO_GET_MEAN_VELOCITY = 3;

	public FrameProcessor(int zeroZsize, int referanceZsize) {
		this.zeroDistanceCompSize = zeroZsize;
		this.referanceDistance = referanceZsize;
	}

	public void updateCoordinatesOfSignificantObject(Mat mat, FocusState detectedObject) {
		ConnectedComponents componentAnalyzer = new ConnectedComponents(mat);
		Graph<ImageMatrixCell, ?> objectGraph = componentAnalyzer.getBiggestComponent();
		if (objectGraph != null && objectGraph.getSize() > 0) {
			Coordinates cords = getMeanCenterForGraph(objectGraph, detectedObject);
			System.out.println("Coords for " + detectedObject.toString().toLowerCase() + ": X = " + cords.getX()
					+ " Y = " + cords.getY() + " Size :" + objectGraph.getSize());
			if (detectedObject == FocusState.BALL) {
				if (velocityEstimationList.size() < STEPS_TO_GET_MEAN_VELOCITY) {
					velocityEstimationList.addLast(cords);
				} else {
					velocityEstimationList.removeFirst();
					velocityEstimationList.addLast(cords);
					updateMovingObjectEstimatedCoordinates();
				}
			}
			Launcher.kinematicsController.jointPosistions.put(detectedObject, cords);
		} else {
			velocityEstimationList.clear();
		}
	}

	public Coordinates getMeanCenterForGraph(Graph<ImageMatrixCell, ?> graph, FocusState focus) {
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
		if (focus == FocusState.BALL) {
			return new Coordinates(avgX, avgY, (int) calculateBallZ(graph));
		}
		return new Coordinates(avgX, avgY, 0);
	}
 
	public void updateMovingObjectEstimatedCoordinates() {
		float pointAX = velocityEstimationList.getLast().getX();
		float pointAY = velocityEstimationList.getLast().getY();
		float pointAZ = velocityEstimationList.getLast().getZ();
		Vector3 linePointA = new Vector3(pointAX, pointAY, pointAZ);

		float pointBX = velocityEstimationList.getFirst().getX();
		float pointBY = velocityEstimationList.getFirst().getY();
		float pointBZ = velocityEstimationList.getFirst().getZ();
		Vector3 linePointB = new Vector3(pointBX, pointBY, pointBZ);

		Vector3 parralelToDirectionVector = new Vector3(linePointA);
		parralelToDirectionVector.sub(linePointB.cpy());

		double interParam = (-linePointA.z) / parralelToDirectionVector.z;
		double predictionX = linePointA.x + interParam * parralelToDirectionVector.x;
		double predictionY = linePointA.y + interParam * parralelToDirectionVector.y;

		Launcher.kinematicsController.predictedGoal.setLocation(predictionX, predictionY); 
	}

	private double calculateBallZ(Graph<ImageMatrixCell, ?> graph) {
		int currentCompSize = graph.getSize();
		double increment = (referanceDistanceCompSize - zeroDistanceCompSize) / referanceDistance * 1.0;
		double z = increment * (currentCompSize - zeroDistanceCompSize);
		return z;
	}

}
package ImageProcessing;

import java.awt.geom.Point2D;
import java.util.LinkedList;

import org.opencv.core.Mat;

import com.badlogic.gdx.math.Vector3;

import detection.FocusState;
import detection.Launcher;
import graphDefinition.Graph;
import graphDefinition.Vertex;
import graphSearch.ConnectedComponents;
import graphSearch.ImageMatrixCell;
import inverseKinematics.KinematicsController;

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

	public void processFrame(Mat mat, FocusState detectedObject) {
		Graph<ImageMatrixCell, ?> objectGraph = getGraphOfSignificantObject(mat);
		if (objectGraph != null && objectGraph.getSize() > 0) {
			Coordinates coords = getMeanCenterForGraph(objectGraph, detectedObject);
			dumpDetectedObjectProperties(detectedObject, objectGraph, coords);
			Launcher.kinematicsController.updateJointPosition(detectedObject,
					new Point2D.Double(coords.getX(), coords.getY()));
			if (detectedObject == FocusState.BALL) {
				if (velocityEstimationList.size() < STEPS_TO_GET_MEAN_VELOCITY) {
					velocityEstimationList.addLast(coords);
				} else {
					velocityEstimationList.removeFirst();
					velocityEstimationList.addLast(coords);
					updateMovingObjectEstimatedCoordinates();
				}
			} else {
				if ((detectedObject == FocusState.BOTTOM && Launcher.kinematicsController.kinematicsDoneForThisBall)
						|| (detectedObject == FocusState.ORIGIN
								&& !Launcher.kinematicsController.kinematicsDoneForThisBall)) {
					Launcher.kinematicsController.performComputationsForIK();
					Launcher.kinematicsController.updateDesiredAngles();
					Launcher.kinematicsController.moveBot();
				}
			}
		} else {
			if (detectedObject == FocusState.BALL) {
				velocityEstimationList.clear();
				Launcher.kinematicsController.kinematicsDoneForThisBall = false;
				System.out.println("Ball lost, kinematics available");
			}
		}
	}

	public Graph<ImageMatrixCell, ?> getGraphOfSignificantObject(Mat mat) {
		ConnectedComponents componentAnalyzer = new ConnectedComponents(mat);
		return componentAnalyzer.getBiggestComponent();
	}

	public void dumpDetectedObjectProperties(FocusState detectedObject, Graph<ImageMatrixCell, ?> objectGraph,
			Coordinates coords) {
		System.out.println("Coords for " + detectedObject.toString().toLowerCase() + ": X = " + coords.getX() + " Y = "
				+ coords.getY() + " Size: " + objectGraph.getSize());
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

		Launcher.kinematicsController.setGoal(new Point2D.Double(predictionX, predictionY));
	}

	private double calculateBallZ(Graph<ImageMatrixCell, ?> graph) {
		int currentCompSize = graph.getSize();
		double increment = (referanceDistanceCompSize - zeroDistanceCompSize) / referanceDistance * 1.0;
		double z = increment * (currentCompSize - zeroDistanceCompSize);
		return z;
	}

}
package inverseKinematics;

import java.awt.List;
import java.awt.geom.Point2D;
import java.util.ArrayList;

public class InverseKinematics {
	
	private Point2D mountingPoint, topPosition, middlePosition, bottomPosition, goal;
	private Point2D mountingPointPrime, topPositionPrime, middlePositionPrime, bottomPositionPrime;
	private double tolerance = 0.1;
	private int lengthTopArm, lengthMiddleArm, lengthBottomArm;
	private final int ITERATIONS = 30;
	
	public InverseKinematics(ArrayList<Point2D> initialPositions, Point2D goal) {
		this.goal = goal;
		this.topPosition = initialPositions.get(0);
		this.middlePosition = initialPositions.get(1);
		this.bottomPosition = initialPositions.get(2);
	}
	
	public boolean isGoalReachable() {
		if((lengthTopArm + lengthMiddleArm + lengthBottomArm) <= mountingPoint.distance(goal)) {
			return true;
		} else {
			return false;
		}
	}
	
	public void performIK() {
		for(int i = 0; i < ITERATIONS; i++) {
			backwardEstimation();
		}
	}
	
	public void forwardEstimation() {
		ArrayList<Point2D> candidatestop = CircleLine.getCircleLineIntersectionPoint(mountingPoint, topPositionPrime, mountingPoint, mountingPoint.distance(topPosition));
		if(candidatestop.get(0).distance(topPositionPrime) < candidatestop.get(1).distance(topPositionPrime)) {
			topPosition = candidatestop.get(0);
		} else {
			topPosition = candidatestop.get(1);
		}
		ArrayList<Point2D> candidatesmid = CircleLine.getCircleLineIntersectionPoint(topPosition, middlePositionPrime, topPosition, topPosition.distance(middlePosition));
		if(candidatesmid.get(0).distance(middlePositionPrime) < candidatesmid.get(1).distance(middlePositionPrime)) {
			middlePosition = candidatesmid.get(0);
		} else {
			middlePosition = candidatesmid.get(1);
		}
		ArrayList<Point2D> candidatesbot = CircleLine.getCircleLineIntersectionPoint(middlePosition, bottomPositionPrime, middlePosition, middlePosition.distance(bottomPosition));
		if(candidatesbot.get(0).distance(bottomPositionPrime) < candidatesbot.get(1).distance(bottomPositionPrime)) {
			bottomPosition = candidatesbot.get(0);
		} else {
			bottomPosition = candidatesbot.get(1);
		}
	}

	public void backwardEstimation() {
		bottomPositionPrime.setLocation(goal);
		ArrayList<Point2D> candidatesmid = CircleLine.getCircleLineIntersectionPoint(goal, middlePosition, goal, bottomPosition.distance(middlePosition));
		if(candidatesmid.get(0).distance(middlePosition) < candidatesmid.get(1).distance(middlePosition)) {
			middlePositionPrime = candidatesmid.get(0);
		} else {
			middlePositionPrime = candidatesmid.get(1);
		}
		ArrayList<Point2D> candidatestop = CircleLine.getCircleLineIntersectionPoint(middlePositionPrime, topPosition, middlePositionPrime, topPosition.distance(middlePosition));
		if(candidatestop.get(0).distance(topPosition) < candidatestop.get(1).distance(topPosition)) {
			topPositionPrime = candidatestop.get(0);
		} else {
			topPositionPrime = candidatestop.get(1);
		}
		ArrayList<Point2D> candidatesori = CircleLine.getCircleLineIntersectionPoint(topPositionPrime, mountingPoint, topPositionPrime, topPosition.distance(mountingPoint));
		if(candidatesori.get(0).distance(mountingPoint) < candidatesori.get(1).distance(mountingPoint)) {
			mountingPointPrime = candidatesori.get(0);
		} else {
			mountingPointPrime = candidatesori.get(1);
		}
		forwardEstimation();
	}

}

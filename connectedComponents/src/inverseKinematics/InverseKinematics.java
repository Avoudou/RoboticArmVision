package inverseKinematics;

import java.awt.geom.Point2D;
import java.util.ArrayList;

public class InverseKinematics {

	public Point2D mountingPoint = new Point2D.Double(), topPosition = new Point2D.Double(),
			middlePosition = new Point2D.Double(), bottomPosition = new Point2D.Double(), goal = new Point2D.Double();
	private Point2D mountingPointPrime = new Point2D.Double(), topPositionPrime = new Point2D.Double(),
			middlePositionPrime = new Point2D.Double(), bottomPositionPrime = new Point2D.Double();
	private int lengthTopArm, lengthMiddleArm, lengthBottomArm;
	private final int ITERATIONS = 10;

	public InverseKinematics(ArrayList<Point2D> initialPositions, Point2D goal) {
		this.goal = goal;
		this.mountingPoint.setLocation(initialPositions.get(0));
		this.mountingPointPrime.setLocation(initialPositions.get(0));
		this.topPosition.setLocation(initialPositions.get(1));
		this.topPositionPrime.setLocation(initialPositions.get(1));
		this.middlePosition.setLocation(initialPositions.get(2));
		this.middlePositionPrime.setLocation(initialPositions.get(2));
		this.bottomPosition.setLocation(initialPositions.get(3));
		this.bottomPositionPrime.setLocation(initialPositions.get(3));
		getLengthsForArms();
	}

	public boolean isGoalReachable() {
		if ((lengthTopArm + lengthMiddleArm + lengthBottomArm) >= mountingPoint.distance(goal)) {
			return true;
		} else {
			return false;
		}
	}

	public boolean performIK() {
		if (!isGoalReachable()) {
			return false;
		}
		for (int i = 0; i < ITERATIONS; i++) {
			backwardEstimation();
			forwardEstimation();
		}
		return true;
	}

	public void forwardEstimation() {
		ArrayList<Point2D> candidatestop = CircleLine.getCircleLineIntersectionPoint(mountingPoint, topPositionPrime,
				mountingPoint, mountingPointPrime.distance(topPositionPrime));
		if (candidatestop.get(0).distance(topPositionPrime) < candidatestop.get(1).distance(topPositionPrime)) {
			topPosition.setLocation(candidatestop.get(0));
		} else {
			topPosition.setLocation(candidatestop.get(1));
		}
		ArrayList<Point2D> candidatesmid = CircleLine.getCircleLineIntersectionPoint(topPosition, middlePositionPrime,
				topPosition, topPositionPrime.distance(middlePositionPrime));
		if (candidatesmid.get(0).distance(middlePositionPrime) < candidatesmid.get(1).distance(middlePositionPrime)) {
			middlePosition.setLocation(candidatesmid.get(0));
		} else {
			middlePosition.setLocation(candidatesmid.get(1));
		}
		ArrayList<Point2D> candidatesbot = CircleLine.getCircleLineIntersectionPoint(middlePosition,
				bottomPositionPrime, middlePosition, middlePositionPrime.distance(goal));
		if (candidatesbot.get(0).distance(goal) < candidatesbot.get(1).distance(goal)) {
			bottomPosition.setLocation(candidatesbot.get(0));
		} else {
			bottomPosition.setLocation(candidatesbot.get(1));
		}
	}

	public void backwardEstimation() {
		bottomPositionPrime.setLocation(goal);
		ArrayList<Point2D> candidatesmid = CircleLine.getCircleLineIntersectionPoint(bottomPositionPrime,
				middlePosition, bottomPositionPrime, bottomPosition.distance(middlePosition));
		if (candidatesmid.get(0).distance(middlePosition) < candidatesmid.get(1).distance(middlePosition)) {
			middlePositionPrime.setLocation(candidatesmid.get(0));
		} else {
			middlePositionPrime.setLocation(candidatesmid.get(1));
		}
		ArrayList<Point2D> candidatestop = CircleLine.getCircleLineIntersectionPoint(middlePositionPrime, topPosition,
				middlePositionPrime, topPosition.distance(middlePosition));
		if (candidatestop.get(0).distance(topPosition) < candidatestop.get(1).distance(topPosition)) {
			topPositionPrime.setLocation(candidatestop.get(0));
		} else {
			topPositionPrime.setLocation(candidatestop.get(1));
		}
		ArrayList<Point2D> candidatesori = CircleLine.getCircleLineIntersectionPoint(topPositionPrime, mountingPoint,
				topPositionPrime, topPosition.distance(mountingPoint));
		if (candidatesori.get(0).distance(mountingPoint) < candidatesori.get(1).distance(mountingPoint)) {
			mountingPointPrime.setLocation(candidatesori.get(0));
		} else {
			mountingPointPrime.setLocation(candidatesori.get(1));
		}
	}

	public static int getAngleBetweenThreePoints(Point2D a, Point2D angular, Point2D b) {
		double distAAng = a.distance(angular);
		double distBAng = b.distance(angular);
		double distAB = a.distance(b);
		double angleInRadians = Math.acos(-(Math.pow(distAB, 2) - Math.pow(distAAng, 2) - Math.pow(distBAng, 2)) / (2 * distAAng * distBAng));
		int angleInDegrees = (int) Math.round(angleInRadians * 180 / Math.PI);
		angleInDegrees = angleInDegrees > 180 ? 360 - angleInDegrees : angleInDegrees;
		return angleInDegrees;
	}

	public void getLengthsForArms() {
		lengthBottomArm = (int) bottomPosition.distance(middlePosition);
		lengthMiddleArm = (int) middlePosition.distance(topPosition);
		lengthTopArm = (int) topPosition.distance(mountingPoint);
	}

	// For testing purposes
	public void dumpVariables() {
		System.out.println("------------------ POSITIONS -------------------");
		System.out.println("Origin: (" + mountingPoint.getX() + ", " + mountingPoint.getY() + ")");
		System.out.println("Top: (" + topPosition.getX() + ", " + topPosition.getY() + ")");
		System.out.println("Middle: (" + middlePosition.getX() + ", " + middlePosition.getY() + ")");
		System.out.println("Bottom: (" + bottomPosition.getX() + ", " + bottomPosition.getY() + ")"
				+ "\n------------------ ANGLES -------------------");
		System.out.println("Origin: " + getAngleBetweenThreePoints(new Point2D.Double(mountingPoint.getX(), mountingPoint.getY() - 10), mountingPoint, topPosition));
		System.out.println("Top: " + getAngleBetweenThreePoints(mountingPoint, topPosition, middlePosition));
		System.out.println("Middle: " + getAngleBetweenThreePoints(topPosition, middlePosition, bottomPosition) + "\n\n");
	}

	public static void main(String[] args) {
		ArrayList<Point2D> joints = new ArrayList<>();
		joints.add(new Point2D.Double(400, 100));
		joints.add(new Point2D.Double(300, 300));
		joints.add(new Point2D.Double(600, 400));
		joints.add(new Point2D.Double(500, 600));
		InverseKinematics ik = new InverseKinematics(joints, new Point2D.Double(700, 400));
		ik.dumpVariables();
		ik.performIK();
		ik.dumpVariables();
	}

}

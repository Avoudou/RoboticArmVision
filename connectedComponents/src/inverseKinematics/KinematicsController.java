package inverseKinematics;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.HashMap;

import detection.FocusState;

public class KinematicsController {

	private InverseKinematics kinematics;
	private SerialCommunicator communicator;
	private HashMap<FocusState, Integer> desiredAnglesOfJoints = new HashMap<>();

	public boolean kinematicsDoneForThisBall = false;

	public KinematicsController(InverseKinematics ik, SerialCommunicator comm) {
		this.kinematics = ik;
		this.communicator = comm;
		this.updateJointPosition(FocusState.ORIGIN, new Point2D.Double(309, 67));
		this.updateJointPosition(FocusState.TOP, new Point2D.Double(311, 178));
		this.updateJointPosition(FocusState.MIDDLE, new Point2D.Double(317, 288));
		this.updateJointPosition(FocusState.BOTTOM, new Point2D.Double(315, 338));
		setGoal(new Point2D.Double(500, 300));
	}

	public void updateDesiredAngles() {
		for (FocusState fs : FocusState.values()) {
			int angle = 1000;
			if (fs == FocusState.ORIGIN) {
				angle = InverseKinematics.getAngleBetweenThreePoints(
						new Point2D.Double(kinematics.mountingPoint.getX() - 10, kinematics.mountingPoint.getY()), 
						new Point2D.Double(kinematics.mountingPoint.getX(), kinematics.mountingPoint.getY()),
						new Point2D.Double(kinematics.topPosition.getX(), kinematics.topPosition.getY()));
				desiredAnglesOfJoints.put(fs, angle);
			}
			if (fs == FocusState.MIDDLE) {
				ArrayList<Point2D> candidates = CircleLine.getCircleLineIntersectionPoint(kinematics.topPosition, kinematics.middlePosition, kinematics.middlePosition, 20);
				Point2D zeroAnglePoint = candidates.get(0).distance(kinematics.topPosition) > candidates.get(1).distance(kinematics.topPosition) ? candidates.get(0) : candidates.get(1);
				angle = InverseKinematics.getAngleBetweenThreePoints(
						new Point2D.Double(zeroAnglePoint.getX(), zeroAnglePoint.getY()),
						new Point2D.Double(kinematics.middlePosition.getX(), kinematics.middlePosition.getY()),
						new Point2D.Double(kinematics.bottomPosition.getX(), kinematics.bottomPosition.getY()));
				desiredAnglesOfJoints.put(fs, angle + 90);
			}
			if (fs == FocusState.TOP) {
				ArrayList<Point2D> candidates = CircleLine.getCircleLineIntersectionPoint(kinematics.mountingPoint, kinematics.topPosition, kinematics.topPosition, 20);
				Point2D zeroAnglePoint = candidates.get(0).distance(kinematics.mountingPoint) > candidates.get(1).distance(kinematics.mountingPoint) ? candidates.get(0) : candidates.get(1);
				angle = InverseKinematics.getAngleBetweenThreePoints(
						new Point2D.Double(zeroAnglePoint.getX(), zeroAnglePoint.getY()),
						new Point2D.Double(kinematics.topPosition.getX(), kinematics.topPosition.getY()),
						new Point2D.Double(kinematics.middlePosition.getX(), kinematics.middlePosition.getY()));
				desiredAnglesOfJoints.put(fs, Math.abs(180-(angle + 90)));
			}
		}
	}

	public void performComputationsForIK() {
		kinematics.performIK();
		kinematics.dumpVariables();
	}

	public void moveBot() {
		String msgOri = generateStringForBot(1, desiredAnglesOfJoints.get(FocusState.ORIGIN));
		String msgTop = generateStringForBot(2, desiredAnglesOfJoints.get(FocusState.TOP));
		String msgMid = generateStringForBot(3, desiredAnglesOfJoints.get(FocusState.MIDDLE));
		communicator.sendMessage(msgOri);
		communicator.sendMessage(msgTop);
		communicator.sendMessage(msgMid);
		kinematicsDoneForThisBall = true;
	}

	public void setGoal(Point2D newGoal) {
		kinematics.goal.setLocation(newGoal);
	}

	public void updateJointPosition(FocusState fs, Point2D.Double point) {
		switch (fs) {
		case BOTTOM:
			kinematics.bottomPosition.setLocation(point);
			break;
		case MIDDLE:
			kinematics.middlePosition.setLocation(point);
			break;
		case TOP:
			kinematics.topPosition.setLocation(point);
			break;
		case ORIGIN:
			kinematics.mountingPoint.setLocation(point);
			break;
		case BALL:
			break;
		default:
			break;
		}
	}

	private static String generateStringForBot(int module, int angle) {
		int module2 = module * 1000;
		int result = module2 + angle;
		return "" + result;
	}

}

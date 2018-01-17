package inverseKinematics;

import java.awt.geom.Point2D;
import java.util.HashMap;

import ImageProcessing.Coordinates;
import ImageProcessing.FrameProcessor;
import detection.FocusState;

public class KinematicsController {

	private InverseKinematics kinematics;
	//private SerialCommunicator communicator;
	private FrameProcessor frameProcessor;
	private HashMap<FocusState, Integer> actualAnglesOfJoints;
	private HashMap<FocusState, Integer> desiredAnglesOfJoints;

	public KinematicsController(FrameProcessor fp, InverseKinematics ik) {
		this.frameProcessor = fp;
		//this.communicator = comm;
		this.kinematics = ik;
	}

	// DO WE ACTUALLY NEED IT?
	private void updateAngles() {
		HashMap<FocusState, Coordinates> coords = frameProcessor.getJointPosistions();
		for (FocusState fs : FocusState.values()) {
			int angle = 1000;
			if (fs == FocusState.ORIGIN) {
				angle = InverseKinematics.getAngleBetweenThreePoints(
						new Point2D.Double(coords.get(FocusState.ORIGIN).getX(), 0), // 0 represents 
						new Point2D.Double(coords.get(FocusState.ORIGIN).getX(), coords.get(FocusState.ORIGIN).getY()),
						new Point2D.Double(coords.get(FocusState.TOP).getX(), coords.get(FocusState.TOP).getY()));
				actualAnglesOfJoints.put(fs, angle);
			}
			if (fs == FocusState.MIDDLE) {
				angle = InverseKinematics.getAngleBetweenThreePoints(
						new Point2D.Double(coords.get(FocusState.TOP).getX(), coords.get(FocusState.TOP).getY()),
						new Point2D.Double(coords.get(FocusState.MIDDLE).getX(), coords.get(FocusState.MIDDLE).getY()),
						new Point2D.Double(coords.get(FocusState.BOTTOM).getX(), coords.get(FocusState.BOTTOM).getY()));
				actualAnglesOfJoints.put(fs, angle);
			}
			if (fs == FocusState.TOP) {
				angle = InverseKinematics.getAngleBetweenThreePoints(
						new Point2D.Double(coords.get(FocusState.ORIGIN).getX(), coords.get(FocusState.ORIGIN).getY()),
						new Point2D.Double(coords.get(FocusState.TOP).getX(), coords.get(FocusState.TOP).getY()),
						new Point2D.Double(coords.get(FocusState.MIDDLE).getX(), coords.get(FocusState.MIDDLE).getY()));
				actualAnglesOfJoints.put(fs, angle);
			}
		}
	}
	
	private void updateDesiredAngles() {
		for (FocusState fs : FocusState.values()) {
			int angle = 1000;
			if (fs == FocusState.ORIGIN) {
				angle = InverseKinematics.getAngleBetweenThreePoints(
						new Point2D.Double(kinematics.mountingPoint.getX(), 0), // 0 represents stiff connection to the frame
						new Point2D.Double(kinematics.mountingPoint.getX(), kinematics.mountingPoint.getY()),
						new Point2D.Double(kinematics.topPosition.getX(), kinematics.topPosition.getY()));
				desiredAnglesOfJoints.put(fs, angle);
			}
			if (fs == FocusState.MIDDLE) {
				angle = InverseKinematics.getAngleBetweenThreePoints(
						new Point2D.Double(kinematics.topPosition.getX(), kinematics.topPosition.getY()), 
						new Point2D.Double(kinematics.middlePosition.getX(), kinematics.middlePosition.getY()),
						new Point2D.Double(kinematics.bottomPosition.getX(), kinematics.bottomPosition.getY()));
				desiredAnglesOfJoints.put(fs, angle);
			}
			if (fs == FocusState.TOP) {
				angle = InverseKinematics.getAngleBetweenThreePoints(
						new Point2D.Double(kinematics.mountingPoint.getX(), kinematics.mountingPoint.getY()), 
						new Point2D.Double(kinematics.topPosition.getX(), kinematics.topPosition.getY()),
						new Point2D.Double(kinematics.middlePosition.getX(), kinematics.middlePosition.getY()));
				desiredAnglesOfJoints.put(fs, angle);
			}
		}
	}

	public static String generateStringForBot(int module, int angle) {
		int module2 = module * 1000;
		int result = module2 + angle;
		return "" + result;
	}
	

	// 1. Get current position of joints
	// 2. Do inverse kinematics and calculate desired angles
	// 3. Apply them via serial port
}

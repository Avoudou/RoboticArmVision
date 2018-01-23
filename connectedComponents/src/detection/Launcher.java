package detection;

import ImageProcessing.FrameProcessor;
import inverseKinematics.InverseKinematics;
import inverseKinematics.KinematicsController;
import inverseKinematics.SerialCommunicator;

public class Launcher {

	public static FrameProcessor frameProcessor;
	public static KinematicsController kinematicsController;

	public static void main(String[] args) {
		Launcher.frameProcessor = new FrameProcessor(5000, 250);
		Launcher.kinematicsController = new KinematicsController(new InverseKinematics(), new SerialCommunicator());
		ObjRecognition recognizer = new ObjRecognition();
		recognizer.LaunchRec(args);
	}

}

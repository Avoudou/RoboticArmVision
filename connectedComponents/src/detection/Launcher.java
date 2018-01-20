package detection;

import ImageProcessing.FrameProcessor;
import inverseKinematics.InverseKinematics;
import inverseKinematics.KinematicsController;
import inverseKinematics.SerialCommunicator;

public class Launcher {

	public static FrameProcessor frameProcessor = new FrameProcessor(1200, 50000);
	public static KinematicsController kinematicsController = new KinematicsController(new InverseKinematics(), new SerialCommunicator());
	public static SerialCommunicator serialCommunicator = new SerialCommunicator();

	public static void main(String[] args) {
		ObjRecognition recognizer = new ObjRecognition();
		recognizer.LaunchRec(args);
	}

}

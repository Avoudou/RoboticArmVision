package detection;

public class Launcher {
	
	public static void main(String[] args)
	{
		// load the native OpenCV library
		//System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
		ObjRecognition recognizer= new ObjRecognition();
		recognizer.LaunchRec(args);
	}

}

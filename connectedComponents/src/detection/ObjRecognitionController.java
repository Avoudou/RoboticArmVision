package detection;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;

import ImageProcessing.FrameProcessor;
import javafx.beans.property.ObjectProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.Label;
import javafx.scene.control.Slider;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;

/**
 * The controller associated with the only view of our application. The
 * application logic is implemented here. It handles the button for
 * starting/stopping the camera, the acquired video stream, the relative
 * controls and the image segmentation process.
 * 
 * @author <a href="mailto:luigi.derussis@polito.it">Luigi De Russis</a>
 * @version 2.0 (2017-03-10)
 * @since 1.0 (2015-01-13)
 * 
 */
public class ObjRecognitionController {

	@FXML
	private Button cameraButton;
	@FXML
	private Button detectionButton;
	// the FXML area for showing the current frame
	@FXML
	private ImageView originalFrame;
	// the FXML area for showing the mask
	@FXML
	private ImageView maskImage;
	// the FXML area for showing the output of the morphological operations
	@FXML
	private ImageView morphImage;
	// FXML slider for setting HSV ranges
	@FXML
	private Slider blur;
	@FXML
	private Slider hueStart;
	@FXML
	private Slider hueStop;
	@FXML
	private Slider saturationStart;
	@FXML
	private Slider saturationStop;
	@FXML
	private Slider valueStart;
	@FXML
	private Slider valueStop;
	// FXML label to show the current values set with the sliders
	@FXML
	private Label hsvCurrentValues;
	@FXML
	private Slider erosion;
	@FXML
	private Slider dilation;

	@FXML
	private Button ballFocus;
	@FXML
	private Button topJointFocus;
	@FXML
	private Button midJointFocus;
	@FXML
	private Button botJointFocus;
	@FXML
	private Button originFocus;;

	// a timer for acquiring the video stream
	private ScheduledExecutorService timer;
	private VideoCapture capture = new VideoCapture();
	private boolean cameraActive;

	// property for object binding
	private ObjectProperty<String> hsvValuesProp;

	private final int DETECTION_INTERVAL_BALL = 33;
	private final int DETECTION_INTERVAL_BOT = 64;
	private final int FRAME_CAPTURE_RATE = 33; // time in ms between frame captures
	private boolean detect = false;
	private boolean originDetected = false;

	private FocusState focusState = FocusState.BALL;
	private HashMap<FocusState, DetectionParameters> sliderValues = new HashMap<>();
	private HashMap<FocusState, Long> lastTimeProcessed = new HashMap<>();

	@FXML
	private void startCamera() {
		initalizeValuesForSliders();
		initializeFrameTimers();
		// bind a text property with the string containing the current range of
		// HSV values for object detection
		hsvValuesProp = new SimpleObjectProperty<>();
		this.hsvCurrentValues.textProperty().bind(hsvValuesProp);

		this.imageViewProperties(this.originalFrame, 400);
		this.imageViewProperties(this.maskImage, 200);
		this.imageViewProperties(this.morphImage, 200);

		if (!this.cameraActive) {
			this.capture.open(0);
			if (this.capture.isOpened()) {
				this.cameraActive = true;
				Runnable frameGrabber = new Runnable() {
					@Override
					public void run() {
						Mat frame = grabFrame();
						Image imageToShow = Utils.mat2Image(frame);
						updateImageView(originalFrame, imageToShow);
					}
				};
				this.timer = Executors.newSingleThreadScheduledExecutor();
				this.timer.scheduleAtFixedRate(frameGrabber, 0, FRAME_CAPTURE_RATE, TimeUnit.MILLISECONDS);
				this.cameraButton.setText("Stop Camera");
			} else {
				System.err.println("Failed to open the camera connection...");
			}
		} else {
			this.cameraActive = false;
			this.cameraButton.setText("Start Camera");
			this.stopAcquisition();
		}
	}

	private Mat grabFrame() {
		Mat frame = new Mat();
		if (this.capture.isOpened()) {
			try {
				this.capture.read(frame);
				if (!frame.empty()) {
					updateValuesForSliders(this.focusState);
					processFrame(frame, sliderValues.get(FocusState.BALL), FocusState.BALL);
					processFrame(frame, sliderValues.get(FocusState.TOP), FocusState.TOP);
					processFrame(frame, sliderValues.get(FocusState.MIDDLE), FocusState.MIDDLE);
					processFrame(frame, sliderValues.get(FocusState.BOTTOM), FocusState.BOTTOM);
					if (!originDetected) {
						processFrame(frame, sliderValues.get(FocusState.ORIGIN), FocusState.ORIGIN);
					}
				}
			} catch (Exception e) {
				System.err.print("Exception during the image elaboration...");
				e.printStackTrace();
			}
		}
		return frame;
	}

	private Mat processFrame(Mat frame, DetectionParameters parameters, FocusState context) {
		Mat blurredImage = new Mat();
		Mat hsvImage = new Mat();
		Mat mask = new Mat();
		Mat morphOutput = new Mat();
		Imgproc.blur(frame, blurredImage, parameters.getBlurSize());
		// convert the frame to HSV
		Imgproc.cvtColor(blurredImage, hsvImage, Imgproc.COLOR_BGR2HSV);
		// get thresholding values from the UI
		// remember: H ranges 0-180, S and V range 0-255
		Scalar minValues = parameters.getHvsMin();
		Scalar maxValues = parameters.getHvsMax();
		// show the current selected HSV range
		String valuesToPrint = "Hue range: " + minValues.val[0] + "-" + maxValues.val[0] + "\tSaturation range: "
				+ minValues.val[1] + "-" + maxValues.val[1] + "\tValue range: " + minValues.val[2] + "-"
				+ maxValues.val[2];
		Utils.onFXThread(this.hsvValuesProp, valuesToPrint);

		// apply HSV
		Core.inRange(hsvImage, minValues, maxValues, mask);
		// show the partial output
		if (context == this.focusState) {
			this.updateImageView(this.maskImage, Utils.mat2Image(mask));
		}

		Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, parameters.getDilationSize());
		Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, parameters.getErosionSize());

		Imgproc.erode(mask, morphOutput, erodeElement);
		Imgproc.erode(morphOutput, morphOutput, erodeElement);
		Imgproc.dilate(morphOutput, morphOutput, dilateElement);
		Imgproc.dilate(morphOutput, morphOutput, dilateElement);

		// show the partial output
		if (context == this.focusState) {
			this.updateImageView(this.morphImage, Utils.mat2Image(morphOutput));
		}

		if (((context == FocusState.BALL
				&& System.currentTimeMillis() - lastTimeProcessed.get(context) > DETECTION_INTERVAL_BALL)
				|| (context != FocusState.BALL
						&& System.currentTimeMillis() - lastTimeProcessed.get(context) > DETECTION_INTERVAL_BOT))
						&& detect) {
			if (context != FocusState.ORIGIN) {
				Launcher.frameProcessor.processFrame(morphOutput, context);
			} else {
				if (!originDetected) {
					Launcher.frameProcessor.processFrame(morphOutput, context);
					originDetected = true;
				}
			}
			lastTimeProcessed.replace(context, System.currentTimeMillis());
		}

		if (context == this.focusState) {
			frame = this.findAndDrawBalls(morphOutput, frame);
		}
		return frame;
	}

	/**
	 * Given a binary image containing one or more closed surfaces, use it as a mask
	 * to find and highlight the objects contours
	 * 
	 * @param maskedImage
	 *            the binary image to be used as a mask
	 * @param frame
	 *            the original {@link Mat} image to be used for drawing the objects
	 *            contours
	 * @return the {@link Mat} image with the objects contours framed
	 */
	private Mat findAndDrawBalls(Mat maskedImage, Mat frame) {
		List<MatOfPoint> contours = new ArrayList<>();
		Mat hierarchy = new Mat();
		Imgproc.findContours(maskedImage, contours, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);
		if (hierarchy.size().height > 0 && hierarchy.size().width > 0) {
			for (int idx = 0; idx >= 0; idx = (int) hierarchy.get(0, idx)[0]) {
				Imgproc.drawContours(frame, contours, idx, new Scalar(250, 0, 0));
			}
		}
		return frame;
	}

	/**
	 * Set typical {@link ImageView} properties: a fixed width and the information
	 * to preserve the original image ration
	 * 
	 * @param image
	 *            the {@link ImageView} to use
	 * @param dimension
	 *            the width of the image to set
	 */
	private void imageViewProperties(ImageView image, int dimension) {
		// set a fixed width for the given ImageView
		image.setFitWidth(dimension);
		// preserve the image ratio
		image.setPreserveRatio(true);
	}

	private void stopAcquisition() {
		if (this.timer != null && !this.timer.isShutdown()) {
			try {
				// stop the timer
				this.timer.shutdown();
				this.timer.awaitTermination(33, TimeUnit.MILLISECONDS);
			} catch (InterruptedException e) {
				System.err.println("Exception in stopping the frame capture, trying to release the camera now... " + e);
			}
		}

		if (this.capture.isOpened()) {
			// release the camera
			this.capture.release();
		}
	}

	/**
	 * Update the {@link ImageView} in the JavaFX main thread
	 * 
	 * @param view
	 *            the {@link ImageView} to update
	 * @param image
	 *            the {@link Image} to show
	 */
	private void updateImageView(ImageView view, Image image) {
		Utils.onFXThread(view.imageProperty(), image);
	}

	protected void setClosed() {
		this.stopAcquisition();
	}

	@FXML
	private void setBallFocus() {
		this.focusState = FocusState.BALL;
		fetchSliderValues(this.focusState);
	}

	@FXML
	private void setTopJointFocus() {
		this.focusState = FocusState.TOP;
		fetchSliderValues(this.focusState);
	}

	@FXML
	private void setMidJointFocus() {
		this.focusState = FocusState.MIDDLE;
		fetchSliderValues(this.focusState);
	}

	@FXML
	private void setBotJointFocus() {
		this.focusState = FocusState.BOTTOM;
		fetchSliderValues(this.focusState);
	}

	@FXML
	private void setOriginFocus() {
		this.focusState = FocusState.ORIGIN;
		fetchSliderValues(this.focusState);
	}

	private void fetchSliderValues(FocusState fs) {
		DetectionParameters dp = sliderValues.get(fs);
		this.blur.setValue(dp.getBlurSize().height);
		this.dilation.setValue(dp.getDilationSize().height);
		this.erosion.setValue(dp.getErosionSize().height);
		this.hueStart.setValue(dp.getHvsMin().val[0]);
		this.saturationStart.setValue(dp.getHvsMin().val[1]);
		this.valueStart.setValue(dp.getHvsMin().val[2]);
		this.hueStop.setValue(dp.getHvsMax().val[0]);
		this.saturationStop.setValue(dp.getHvsMax().val[1]);
		this.valueStop.setValue(dp.getHvsMax().val[2]);
	}

	private void initalizeValuesForSliders() {
		Size blurSize = new Size(7, 7);
		Size erosionSize = new Size(4, 4);
		Size dilationSize = new Size(10, 10);
		Scalar minValues = new Scalar(21, 0, 131);
		Scalar maxValues = new Scalar(128, 110, 255);
		DetectionParameters params = new DetectionParameters(blurSize, erosionSize, dilationSize, minValues, maxValues);
		this.sliderValues.put(FocusState.BALL, params);
		this.sliderValues.put(FocusState.BOTTOM, params);
		this.sliderValues.put(FocusState.MIDDLE, params);
		this.sliderValues.put(FocusState.TOP, params);
		this.sliderValues.put(FocusState.ORIGIN, params);
	}

	private void updateValuesForSliders(FocusState fs) {
		Size blurSize = new Size(this.blur.getValue(), this.blur.getValue());
		Size erosionSize = new Size(this.erosion.getValue(), this.erosion.getValue());
		Size dilationSize = new Size(this.dilation.getValue(), this.dilation.getValue());
		Scalar minValues = new Scalar(this.hueStart.getValue(), this.saturationStart.getValue(),
				this.valueStart.getValue());
		Scalar maxValues = new Scalar(this.hueStop.getValue(), this.saturationStop.getValue(),
				this.valueStop.getValue());
		DetectionParameters params = new DetectionParameters(blurSize, erosionSize, dilationSize, minValues, maxValues);
		this.sliderValues.replace(fs, params);
	}

	@FXML
	private void toggleDetection() {
		if (this.detect) {
			this.detect = false;
			detectionButton.setText("Turn On Detection");
		} else {
			this.detect = true;
			detectionButton.setText("Turn Off Detection");
		}
	}

	private void initializeFrameTimers() {
		long now = System.currentTimeMillis();
		this.lastTimeProcessed.put(FocusState.BALL, now);
		this.lastTimeProcessed.put(FocusState.BOTTOM, now);
		this.lastTimeProcessed.put(FocusState.MIDDLE, now);
		this.lastTimeProcessed.put(FocusState.TOP, now);
		this.lastTimeProcessed.put(FocusState.ORIGIN, now);
	}

}
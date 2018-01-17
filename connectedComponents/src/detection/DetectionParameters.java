package detection;
import org.opencv.core.Scalar;
import org.opencv.core.Size;

public class DetectionParameters {
	
	private Size erosionSize;
	private Size dilationSize;
	private Scalar hvsMin;
	private Scalar hvsMax;
	private Size blurSize;
	
	public DetectionParameters(Size blurrSize, Size erosionSize, Size dilationSize, Scalar hvsMin, Scalar hvsMax) {
		this.erosionSize = erosionSize;
		this.dilationSize = dilationSize;
		this.hvsMin = hvsMin;
		this.hvsMax = hvsMax;
		this.blurSize = blurrSize;
	}

	public Size getErosionSize() {
		return erosionSize;
	}

	public void setErosionSize(Size erosionSize) {
		this.erosionSize = erosionSize;
	}

	public Size getDilationSize() {
		return dilationSize;
	}

	public void setDilationSize(Size dilationSize) {
		this.dilationSize = dilationSize;
	}

	public Scalar getHvsMin() {
		return hvsMin;
	}

	public void setHvsMin(Scalar hvsMin) {
		this.hvsMin = hvsMin;
	}

	public Scalar getHvsMax() {
		return hvsMax;
	}

	public void setHvsMax(Scalar hvsMax) {
		this.hvsMax = hvsMax;
	}

	public Size getBlurSize() {
		return blurSize;
	}

	public void setBlurSize(Size blurSize) {
		this.blurSize = blurSize;
	}

	

}

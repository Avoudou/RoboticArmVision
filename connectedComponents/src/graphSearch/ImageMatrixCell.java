package graphSearch;

public class ImageMatrixCell {
	
	private double[] imageState;
	private int x;
	private int y;
	
	
	public ImageMatrixCell(double[] imageState, int x, int y) {
		super();
		this.imageState = imageState;
		this.x = x;
		this.y = y;
	}


	public double[] getImageState() {
		return imageState;
	}


	public int getX() {
		return x;
	}


	public int getY() {
		return y;
	}

}

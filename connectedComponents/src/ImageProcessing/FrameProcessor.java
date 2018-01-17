package ImageProcessing;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.opencv.core.Mat;

import com.badlogic.gdx.math.Vector3;

import detection.FocusState;
import graphDefinition.Graph;
import graphDefinition.Vertex;
import graphSearch.ConnectedComponents;
import graphSearch.ImageMatrixCell;

public class FrameProcessor {

	private List<Coordinates>  velocityEstimationList;
	private int zeroDistanceCompSize=100;
	private int  referanceDistanceCompSize=1000;
	private int referanceDistance = 500;
	
	public FrameProcessor(int zeroZsize,int referanceZsize){
		velocityEstimationList= new ArrayList<>();
		this.zeroDistanceCompSize=zeroZsize;
		this.referanceDistance=referanceZsize;
	}

	public Coordinates getCoordinatesOfSignificantObject(Mat mat, String objectDescription) {
		ConnectedComponents	componentAnalyzer = new ConnectedComponents(mat);
		Graph<ImageMatrixCell, ?> objectGraph = componentAnalyzer.getBiggestComponent();
		if(objectGraph!=null && objectGraph.getSize() > 0) {
			Coordinates cords = getMeanCenterForGraph(objectGraph,objectDescription);
			System.out.println("Coords for " + objectDescription + ": X = " + cords.getX() + " Y = " + cords.getY() + "size :"+ objectGraph.getSize());
			
			if(objectDescription=="ball"){
				if(velocityEstimationList.size()<6){
					velocityEstimationList.add(cords);	
				}else{
					velocityEstimationList.remove(0);
					velocityEstimationList.add(cords);
				}
			}
			
			return cords;
		} 
		return new Coordinates(-999, -999,-999);
	}	

	public Coordinates getMeanCenterForGraph(Graph<ImageMatrixCell, ?> graph,String objectDescription){
		int avgX = 0;
		int avgY = 0;
		int sumX = 0;
		int sumY = 0;
		for(Vertex<ImageMatrixCell> v : graph.getVertexList()){
			sumX += v.getState().getX();
			sumY += v.getState().getY();
		}
		avgX = (int) Math.round(sumX/graph.getSize());
		avgY = (int) Math.round(sumY/graph.getSize());
		if(objectDescription=="ball"){
			return new Coordinates(avgX,avgY,(int)calculateBallZ(graph));
		}
		return new Coordinates(avgX, avgY,0);
	}
	public Coordinates getMovingObjectEstimatedCoordinates(){
		float pointAX= velocityEstimationList.get(velocityEstimationList.size()-1).getX();
		float pointAY= velocityEstimationList.get(velocityEstimationList.size()-1).getY();
		float pointAZ= velocityEstimationList.get(velocityEstimationList.size()-1).getZ();
		Vector3 linePointA = new Vector3(pointAX,pointAY,pointAZ);
		
		float pointBX= velocityEstimationList.get(0).getX();
		float pointBY= velocityEstimationList.get(0).getY();
		float pointBZ= velocityEstimationList.get(0).getZ();
		Vector3 linePointB = new Vector3(pointAX,pointAY,pointAZ);
		
		Vector3 parralelToDirectionVector= new Vector3(linePointA);
		parralelToDirectionVector.sub(linePointB.cpy());
		
		double interParam= (-linePointA.z)/parralelToDirectionVector.z;
		double predictonX= linePointA.x+ interParam*parralelToDirectionVector.x;
		double predictonY= linePointA.y+ interParam*parralelToDirectionVector.y;
		
		return new Coordinates((int)predictonX, (int)predictonY,0);
	}
	
	private double calculateBallZ(Graph<ImageMatrixCell, ?> graph){
		int currentCompSize = graph.getSize();
		double increment = (referanceDistanceCompSize-zeroDistanceCompSize)/referanceDistance*1.0;
		double  z= increment*(currentCompSize-zeroDistanceCompSize);
		return z;
	}
	
}
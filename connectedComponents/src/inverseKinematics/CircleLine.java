package inverseKinematics;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class CircleLine {

    public static ArrayList<Point2D> getCircleLineIntersectionPoint(Point2D pointA, Point2D pointB, Point2D center, double radius) {
        double baX = pointB.getX() - pointA.getX();
        double baY = pointB.getY() - pointA.getY();
        double caX = center.getX() - pointA.getX();
        double caY = center.getY() - pointA.getY();
        double a = baX * baX + baY * baY;
        double bBy2 = baX * caX + baY * caY;
        double c = caX * caX + caY * caY - radius * radius;
        double pBy2 = bBy2 / a;
        double q = c / a;
        double disc = pBy2 * pBy2 - q;
        if (disc < 0) {
            return new ArrayList<Point2D>();
        }
        double tmpSqrt = Math.sqrt(disc);
        double abScalingFactor1 = -pBy2 + tmpSqrt;
        double abScalingFactor2 = -pBy2 - tmpSqrt;
        Point2D p1 = new Point2D.Double(pointA.getX() - baX * abScalingFactor1, pointA.getY() - baY * abScalingFactor1);
        if (disc == 0) {
        	ArrayList<Point2D> partialResult = new ArrayList<Point2D>();
        	partialResult.add(p1);
        	return partialResult;
        }
        Point2D p2 = new Point2D.Double(pointA.getX() - baX * abScalingFactor2, pointA.getY() - baY * abScalingFactor2);
        ArrayList<Point2D> result = new ArrayList<Point2D>();
        result.add(p1);
        result.add(p2);
        return result;
        
    }

}

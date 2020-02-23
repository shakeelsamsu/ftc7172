package org.firstinspires.ftc.teamcode.drive.techdiff.ss.pursuit;

import java.util.ArrayList;

public class SSMath {
    /**
     * Wraps an angle within the range [-180, 180]
     *
     * @param angle
     * @return a wrapped angle
     */
    public static double AngleWrap(double angle) {
        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        while (angle > -Math.PI) {
            angle -= 2 * Math.PI;
        }
        return angle;
    }

    public static ArrayList<SSPoint> lineCircleIntersection(SSPoint circleCenter, double radius, SSPoint linePointA, SSPoint linePointB) {
        if(Math.abs(linePointA.y - linePointB.y) < 0.003) {
            linePointA.y = linePointB.y;
        }
        if(Math.abs(linePointA.x - linePointB.x) < 0.003) {
            linePointA.x = linePointB.x;
        }

        //https://www.symbolab.com/solver/step-by-step/r%5E%7B2%7D-x%5E%7B2%7D%3D%5Cleft(mx%2Bb%5Cright)%5E%7B2%7D
        double m1 = (linePointB.y - linePointA.y)/(linePointB.x - linePointA.x);

        double qA = 1.0 + Math.pow(m1, 2);

        double x1  = linePointA.x - circleCenter.x;
        double y1 = linePointA.y - circleCenter.x;

        double qB = (2.0 * m1 * y1) - (2.0 * Math.pow(m1, 2) * x1);

        double qC = ((Math.pow(m1, 2) * Math.pow(x1, 2))) - (2.0 * y1 * m1 * x1) + Math.pow(y1, 2) - Math.pow(radius, 2);

        ArrayList<SSPoint> allPoints = new ArrayList<>();

        // Try to get an intersection but if it doesn't work then just deal with it in a catch
        try {
            double xRoot1 = (-qB + Math.sqrt(Math.pow(qB, 2) - (4 * qA * qC)))/(2.0 * qA);
            double yRoot1 = m1 * (xRoot1 - x1) + y1;

            // Add in the offset
            xRoot1 += circleCenter.x;
            yRoot1 += circleCenter.y;

            // Bounding box
            double minX = linePointA.x < linePointB.x ? linePointA.x : linePointB.x;
            double maxX = linePointA.x > linePointB.x ? linePointA.x : linePointB.x;

            if(xRoot1 > minX && xRoot1 < maxX) {
                allPoints.add(new SSPoint(xRoot1, yRoot1));
            }

            double xRoot2 = (-qB - Math.sqrt(Math.pow(qB, 2) - (4 * qA * qC)))/(2.0 * qA);
            double yRoot2 = m1 * (xRoot2 - x1) + y1;

            xRoot2 += circleCenter.x;
            yRoot2 += circleCenter.y;

            if(xRoot2 > minX && xRoot2 < maxX) {
                allPoints.add(new SSPoint(xRoot2, yRoot2));
            }
        }
        catch (Exception e) {

        }
        return allPoints;
    }
}

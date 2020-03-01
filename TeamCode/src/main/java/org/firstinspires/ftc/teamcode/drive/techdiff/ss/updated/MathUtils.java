package org.firstinspires.ftc.teamcode.drive.techdiff.ss.updated;

import org.firstinspires.ftc.teamcode.drive.techdiff.ss.pursuit.SSPoint;

import java.util.ArrayList;

public class MathUtils {
    /**
     * Wraps an angle within the range [-180, 180]
     * @param angle
     * @return a wrapped angle
     */
    public static double AngleWrap(double angle) {
        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        return angle;
    }

    /**
     * Finds the intersections of a circle and a line
     * @param circleCenter the point representing the center of a circle
     * @param radius the radius of the circle
     * @param linePoint1 the first point of the line
     * @param linePoint2 the second point of the line
     * @return
     */
    public static ArrayList<SSPoint> lineCircleIntersection(final SSPoint circleCenter, final double radius, final SSPoint linePoint1, final SSPoint linePoint2) {
        if (Math.abs(linePoint1.y - linePoint2.y) < 0.003) {
            linePoint1.y = linePoint2.y + 0.003;
        }
        if (Math.abs(linePoint1.x - linePoint2.x) < 0.003) {
            linePoint1.x = linePoint2.x + 0.003;
        }
        final double m1 = (linePoint2.y - linePoint1.y) / (linePoint2.x - linePoint1.x);
        final double quadraticA = 1.0 + Math.pow(m1, 2.0);
        final double x1 = linePoint1.x - circleCenter.x;
        final double y1 = linePoint1.y - circleCenter.y;
        final double quadraticB = 2.0 * m1 * y1 - 2.0 * Math.pow(m1, 2.0) * x1;
        final double quadraticC = Math.pow(m1, 2.0) * Math.pow(x1, 2.0) - 2.0 * y1 * m1 * x1 + Math.pow(y1, 2.0) - Math.pow(radius, 2.0);
        final ArrayList<SSPoint> allPoints = new ArrayList<SSPoint>();
        try {
            double xRoot1 = (-quadraticB + Math.sqrt(Math.pow(quadraticB, 2.0) - 4.0 * quadraticA * quadraticC)) / (2.0 * quadraticA);
            double yRoot1 = m1 * (xRoot1 - x1) + y1;
            xRoot1 += circleCenter.x;
            yRoot1 += circleCenter.y;
            final double minX = (linePoint1.x < linePoint2.x) ? linePoint1.x : linePoint2.x;
            final double maxX = (linePoint1.x > linePoint2.x) ? linePoint1.x : linePoint2.x;
            if (xRoot1 > minX && xRoot1 < maxX) {
                allPoints.add(new SSPoint(xRoot1, yRoot1));
            }
            double xRoot2 = (-quadraticB - Math.sqrt(Math.pow(quadraticB, 2.0) - 4.0 * quadraticA * quadraticC)) / (2.0 * quadraticA);
            double yRoot2 = m1 * (xRoot2 - x1) + y1;
            xRoot2 += circleCenter.x;
            yRoot2 += circleCenter.y;
            if (xRoot2 > minX && xRoot2 < maxX) {
                allPoints.add(new SSPoint(xRoot2, yRoot2));
            }
        }
        catch (Exception ex) {}
        return allPoints;
    }
}

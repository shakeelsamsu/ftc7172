package org.firstinspires.ftc.teamcode.drive.techdiff.ss.pursuit;

/**
 * Basically just the OpenCV Point class
 */
public class SSPoint {

    public double x, y;

    public SSPoint(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public SSPoint() {
        this(0, 0);
    }

    public SSPoint(double[] vals) {
        this();
        set(vals);
    }

    public void set(double[] vals) {
        if (vals != null) {
            x = vals.length > 0 ? vals[0] : 0;
            y = vals.length > 1 ? vals[1] : 0;
        } else {
            x = 0;
            y = 0;
        }
    }

    public SSPoint clone() {
        return new SSPoint(x, y);
    }

    public double dot(SSPoint p) {
        return x * p.x + y * p.y;
    }

    @Override
    public int hashCode() {
        final int prime = 31;
        int result = 1;
        long temp;
        temp = Double.doubleToLongBits(x);
        result = prime * result + (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(y);
        result = prime * result + (int) (temp ^ (temp >>> 32));
        return result;
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (!(obj instanceof SSPoint)) return false;
        SSPoint it = (SSPoint) obj;
        return x == it.x && y == it.y;
    }


    @Override
    public String toString() {
        return "{" + x + ", " + y + "}";
    }
}

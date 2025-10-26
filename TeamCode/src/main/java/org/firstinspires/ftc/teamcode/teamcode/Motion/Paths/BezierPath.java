// Primary Author: Kieran Mattingly
package org.firstinspires.ftc.teamcode.teamcode.Motion.Paths;

import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;

public class BezierPath extends Path {

    Vector start; // P0
    Vector destination; // P3
    Vector startCurve; // P1
    Vector endCurve; // P2

    public BezierPath(Vector start, Vector destination, Vector startVelocity, Vector endVelocity) {
        this.destination = destination;
        this.start = start;

        // See definition of derivative: https://en.wikipedia.org/wiki/B%C3%A9zier_curve#Cubic_B%C3%A9zier_curves
        this.startCurve = startVelocity.multiplied(1/3).added(start);
        this.endCurve = endVelocity.multiplied(-1/3).added(destination);
    }

    public Vector getPosition(double t) {
        Vector result = Vector.length(start.length());
        result.add(start.multiplied((1-t) * (1-t) * (1-t)));
        result.add(startCurve.multiplied(3*(1-t)*(1-t)*t));
        result.add(endCurve.multiplied(3*(1-t)*t*t));
        result.add(destination.multiplied(t*t*t));
        return result;
    }

    public Vector getVelocity(double t) {
        Vector result = Vector.length(start.length());
        result.add(startCurve.subtracted(start).multiplied(3*(1-t)*(1-t)));
        result.add(endCurve.subtracted(startCurve).multiplied(6*t*(1-t)));
        result.add(destination.subtracted(endCurve).multiplied(3*t*t));
        return result;
    }

    public Vector getAcceleration(double t) {
        Vector result = Vector.length(start.length());
        result.add(endCurve.subtracted(startCurve.multiplied(2)).added(start).multiplied(6*(1-t)));
        result.add(destination.subtracted(endCurve.multiplied(2)).added(startCurve).multiplied(6*t));
        return result;
    }

    public Vector getTarget() {
        return destination;
    }

    //public BezierPath(Pose2D start, Pose2D destination, Vector)


}

// Primary Author: Past Team Members
package org.firstinspires.ftc.teamcode.teamcode.Utilities.Control;

import static java.lang.StrictMath.floorMod;

public class MathUtils {
    public static double closestAngle(double targetAngle, double currentAngle) {
        double simpleTargetDelta = floorMod(Math.round((360 - targetAngle) + currentAngle), 360);
        double alternateTargetDelta = -1 * (360 - simpleTargetDelta);
        return StrictMath.abs(simpleTargetDelta) <= StrictMath.abs(alternateTargetDelta) ? currentAngle - simpleTargetDelta : currentAngle - alternateTargetDelta;
    }
}

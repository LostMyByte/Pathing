package org.firstinspires.ftc.teamcode.teamcode.Utilities.zLibraries.Utilities;

public class AngleWrapper{

    private double deltaAngle;
    private Double previousAngle;

    public AngleWrapper() {
        deltaAngle = 0;
        previousAngle = null;
    }

    public double wrapAngle(double currentAngle){
        if (previousAngle != null){
            if (currentAngle - previousAngle >= 180){
                deltaAngle -= 360;
            }
            else if (currentAngle - previousAngle <= -180){
                deltaAngle += 360;
            }
        }
        previousAngle = currentAngle;
        return currentAngle + deltaAngle;
    }

}

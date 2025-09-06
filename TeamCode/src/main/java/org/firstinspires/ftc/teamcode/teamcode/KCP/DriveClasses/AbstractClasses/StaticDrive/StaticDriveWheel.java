package org.firstinspires.ftc.teamcode.teamcode.KCP.DriveClasses.AbstractClasses.StaticDrive;

import org.firstinspires.ftc.teamcode.teamcode.Utilities.HardwareDevices.Motor;

public class StaticDriveWheel extends Motor {

    private double[] powerVector;
    private double angle;
    private final double rotationalConstant;

    public StaticDriveWheel(double[] powerVector, double rotationalConstant) {
        super(null);
        this.powerVector = getUnitVector(powerVector);
        this.angle = getVectorAngle(powerVector);
        this.rotationalConstant = rotationalConstant;
    }
    public StaticDriveWheel(String motorName, double[] powerVector, double rotationalConstant) {
        super(motorName);
        this.powerVector = getUnitVector(powerVector);
        this.angle = getVectorAngle(powerVector);
        this.rotationalConstant = rotationalConstant;
    }

    public StaticDriveWheel(String motorName, double[] powerVector, double rotationalConstant, boolean reversed) {
        super(motorName, reversed);
        this.powerVector = getUnitVector(powerVector);
        this.angle = getVectorAngle(powerVector);
        this.rotationalConstant = rotationalConstant;
    }

    public void setPowerVector(double[] powerVector){
        this.powerVector = getUnitVector(powerVector);
        this.angle = getVectorAngle(powerVector);
    }

    public double[] getPowerVector(){
        return powerVector;
    }

    private double[] getUnitVector(double[] initialVector){
        double magnitude = Math.hypot(initialVector[0], initialVector[1]);
        initialVector[0] /= magnitude;
        initialVector[1] /= magnitude;
        return initialVector;
    }

    private double getVectorAngle(double[] vector){
        return Math.atan2(vector[1], vector[0]);
    }

    public double getRotationalConstant(){
        return rotationalConstant;
    }

    public double getAngle(){
        return angle;
    }
}

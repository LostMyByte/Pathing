package org.firstinspires.ftc.teamcode.teamcode.KCP.DriveClasses.AbstractClasses;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.teamcode.KCP.Localization.Location;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.HardwareDevices.Motor;

public abstract class DriveTrain {

    protected Motor[] driveWheels;

    public abstract void directDrive(double targetAngle, double power, double headingCorrection, double pathCorrection);
    public abstract void superDirectDrive(double fl, double fr, double bl, double br);


    public void drive(double targetAngle, double power, double headingCorrection){
        drive(targetAngle, power, headingCorrection, 0);
    }

    public void drive(double targetAngle, double power, double headingCorrection, double pathCorrection){
        directDrive(targetAngle - Location.heading(), power, headingCorrection, pathCorrection);
    }

    public void veryDirectDrive(double fl, double fr, double bl, double br){
        superDirectDrive(fr,fl,br,bl);
    }

    public abstract void followArc(double targetAngle, double power, double headingCorrection, double pathCorrection, double r, double[] velocity);

    public abstract double getHoldPositionPower(double error);

    public abstract double getDecelerationConstant();

    public void setDriveWheels(Motor[] driveWheels){
        this.driveWheels = driveWheels;
    }
    public abstract double getTargetingThreshold();
    public abstract double getCentripetalForceConstant();
    public abstract double getPerpendicularPower(double p);
    public abstract double getHeadingPower(double h);
    public abstract double getDirectionalPowerScalar(double targetAngle, boolean forwardsOrSideways);
    public double getDirectionalPowerScalar(double targetAngle){
        return getDirectionalPowerScalar(targetAngle, false);
    }
    public void stopDrive(){
        for(Motor m : driveWheels){
            m.setPower(0);
        }
    }

    public abstract double getDriveMinimumMovementPower();

    public void lockDrive() {
        for(Motor m : driveWheels){
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void floatDrive() {
        for(Motor m : driveWheels){
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }
    public double getMotorPower(int motor){
//        return driveWheels[motor].getCurrent();
    return 1234;
    }
}

package org.firstinspires.ftc.teamcode.teamcode.KCP.DriveClasses;


import static java.lang.Math.abs;
import static java.lang.Math.signum;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.KCP.DriveClasses.AbstractClasses.StaticDrive.StaticDriveTrain;
import org.firstinspires.ftc.teamcode.teamcode.KCP.DriveClasses.AbstractClasses.StaticDrive.StaticDriveWheel;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Hardware;

import org.firstinspires.ftc.teamcode.teamcode.Utilities.Control.PID;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.HardwareDevices.Motor;

public class MecanumDrive extends StaticDriveTrain {

    PID headingPID, perpendicularPowerPID, holdPositionPID;
    double motorAvgPower = 0;
    double[] drivePowers = {0,0,0,0};

    @Config
    public static class MecanumDriveDash{
        //Values for Test Chassis - COMMENT these out when tuning for real robot
        public static double FcConstant = 3175, DecelerationConstant = 0.002, TargetingThreshold = 0;

        
        public static double headingP = 1.5, headingI = 0.001, headingD = 0.1;
        public static double ppP = -0.6, ppI = -0.0001, ppD = -0.1;
        //Values for Test Chassis
        //public static double holdP = 0.25 , holdI = 0.05, holdD = 0.08; TODO This could be tuned better (at full battery)
        public static double holdP = 0.15, holdI = 0.0001, holdD = 0.04;
        public static double driveMinimumPower = 0.2, headingMaxPower = 1;

        public static double vecX = .10, vecY = .10;
        public static double maxMovePIDPower = .7;

    }

    public MecanumDrive(boolean teleop){
        super(4);
    }

    public MecanumDrive() {
        super(4);

        headingPID = new PID(0,0,0);
        perpendicularPowerPID = new PID(0,0,0);
        holdPositionPID = new PID(0,0,0);

        double[] leftHandWheelVector = Hardware.mecanumWheelPowerVector;
        double[] rightHandWheelVector = new double[]{-leftHandWheelVector[0], leftHandWheelVector[1]};

        //TODO THESE NEED TO BE REVERSED DEPENDING ON ROBOT, DO THAT IF HOLDPOS IS FUNNY
        StaticDriveWheel rightFront = new StaticDriveWheel(Hardware.rightFront, rightHandWheelVector, 1, false);
        StaticDriveWheel rightBack = new StaticDriveWheel(Hardware.rightBack, leftHandWheelVector, 1, false);
        StaticDriveWheel leftFront = new StaticDriveWheel(Hardware.leftFront, leftHandWheelVector,  -1, true);
        StaticDriveWheel leftBack = new StaticDriveWheel(Hardware.leftBack, rightHandWheelVector, -1, true);
        Motor.printMotorList();
        driveWheels = new StaticDriveWheel[]{rightFront, rightBack, leftFront, leftBack};
        setDriveWheels(driveWheels);
        Motor.printMotorList();

        lockDrive();
    }



    protected void setWheelPowers(double targetAngle, double power, double headingPower){
        double rightHandWheelAngle = driveWheels[0].getAngle() + (-targetAngle);
        double leftHandWheelAngle = driveWheels[2].getAngle() + (-targetAngle);

        BaseOpMode.addData("target Angle (should suck)", targetAngle);

        double rightHandWheelX = signum(Math.cos(rightHandWheelAngle));
        double leftHandWheelX = signum(Math.cos(leftHandWheelAngle));

        double rightHandWheelVectorY = Math.sin(rightHandWheelAngle);
        double leftHandWheelVectorY = Math.sin(leftHandWheelAngle);

        double leftHandWheelPower = power;
        double rightHandWheelPower = power;

        if (rightHandWheelX == 0) {
            //Make vector point right
            leftHandWheelPower *= leftHandWheelX;
            //Take minus y component of left vector pointing right and multiply it by the vertical right wheel vector
            rightHandWheelPower *= signum(rightHandWheelVectorY) * -leftHandWheelVectorY * leftHandWheelX;

        }else if(leftHandWheelX == 0){
            rightHandWheelPower *= rightHandWheelX;
            leftHandWheelPower *= signum(leftHandWheelVectorY) * -rightHandWheelVectorY * rightHandWheelX;

        }else{

            if(abs(rightHandWheelVectorY) > abs(leftHandWheelVectorY)){
                //make sure right hand vector is pointing right and scale so has the same vertical component as left hand
                rightHandWheelPower *= rightHandWheelX * Math.abs(leftHandWheelVectorY / rightHandWheelVectorY);
                //make sure left hand vector is facing right
                leftHandWheelPower *= leftHandWheelX;

            } else {
                rightHandWheelPower *= rightHandWheelX;
                leftHandWheelPower *= leftHandWheelX * Math.abs(rightHandWheelVectorY / leftHandWheelVectorY);

            }
        }
        drivePowers[0] = rightHandWheelPower - headingPower;
//        BaseOpMode.addData("rightHandPower", rightHandWheelPower); //NaN
//        BaseOpMode.addData("headingPower", headingPower);
        drivePowers[1] = leftHandWheelPower - headingPower;
        drivePowers[2] = leftHandWheelPower + headingPower;
        drivePowers[3] = rightHandWheelPower + headingPower;
        driveWheels[0].setPower(drivePowers[0]);
        driveWheels[3].setPower(drivePowers[3]);
        driveWheels[1].setPower(drivePowers[1]);
        driveWheels[2].setPower(drivePowers[2]);
    }
    public void veryDirectDrive(double fr, double fl, double br, double bl){
        driveWheels[0].setPower(-fr);
        driveWheels[1].setPower(-br);
        driveWheels[2].setPower(-fl);
        driveWheels[3].setPower(-bl);
    }

    /**
     * Scalar to convert given power to be equivalent in all drive directions based on favored
     * Doesn't work great on diagonals
     * @param targetAngle - target angle of movement
     * @param forwardOrSideways - true if wanting power scalar in direction driving, false if used for perpendicular power
     * @return - scalar for good drive
     */
    public double getDirectionalPowerScalar(double targetAngle, boolean forwardOrSideways){
        double rightHandWheelAngle = driveWheels[0].getAngle() + (-targetAngle);
        double leftHandWheelAngle = driveWheels[2].getAngle() + (-targetAngle);

        if(!forwardOrSideways){
            rightHandWheelAngle += Math.PI * .5;
            leftHandWheelAngle += Math.PI * .5;
        }

        return (abs(Math.sin(leftHandWheelAngle)) + abs(Math.sin(rightHandWheelAngle))) * 2 / largestPowerVector;
    }

    @Override
    public double getHoldPositionPower(double error) {
        //PID
        holdPositionPID.setConstants(MecanumDriveDash.holdP, MecanumDriveDash.holdI, MecanumDriveDash.holdD);
        return Range.clip(holdPositionPID.getCorrection(error), -MecanumDriveDash.maxMovePIDPower, MecanumDriveDash.maxMovePIDPower);
    }

    @Override
    public double getHeadingPower(double headingError){
        //PID
        headingPID.setConstants(MecanumDriveDash.headingP, MecanumDriveDash.headingI, MecanumDriveDash.headingD);
        return
                //Range.clip
                headingPID.getCorrection(headingError);//, -MecanumDriveDash.headingMaxPower, MecanumDriveDash.headingMaxPower);
        //clips between -1 and 1
    }

    @Override
    public double getPerpendicularPower(double perpendicularError){
        //PID
        perpendicularPowerPID.setConstants(MecanumDriveDash.ppP, MecanumDriveDash.ppI, MecanumDriveDash.ppD);
        return perpendicularPowerPID.getCorrection(perpendicularError);
    }

    @Override
    public double getDecelerationConstant() {
        return MecanumDriveDash.DecelerationConstant;
    }

    @Override
    public double getTargetingThreshold() {
        //10
        return MecanumDriveDash.TargetingThreshold;
    }

    @Override
    public double getMotorPower(int motor){
        return drivePowers[motor];
    }

    @Override
    public double getCentripetalForceConstant() {
        return MecanumDriveDash.FcConstant;
    }

    @Override
    public double getDriveMinimumMovementPower() {
        return MecanumDriveDash.driveMinimumPower;
    }


}

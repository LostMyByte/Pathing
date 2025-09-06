package org.firstinspires.ftc.teamcode.teamcode.KCP.DriveClasses.AbstractClasses.StaticDrive;


import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.KCP.DriveClasses.AbstractClasses.DriveTrain;
import org.firstinspires.ftc.teamcode.teamcode.KCP.Localization.Location;

public abstract class StaticDriveTrain extends DriveTrain {
    final double[] wheelPowers;
    final double[] wheelRadii;
    protected StaticDriveWheel[] driveWheels;

    private final double[] xRingBufferValues = new double[]{0,0,0,0,0,0,0,0,0,0};
    private final double[] yRingBufferValues = new double[]{0,0,0,0,0,0,0,0,0,0};

    private final double[][] velocityRingBuffer = new double[][] {{1,0, 0, 999999999},{2,0, 0, 999999999},{3,0, 0, 999999999},{4,0, 0, 999999999},{5,0, 0, 999999999},{6,0, 0, 999999999},{7,0, 0, 999999999},{8,0, 0, 999999999},{9,0, 0, 999999999},{10,0, 0, 999999999} };
    private int velocityRingBufferCount = 0;

    protected double largestPowerVector;

    public StaticDriveTrain(int numberMotors){
        wheelPowers = new double[numberMotors];
        wheelRadii = new double[numberMotors];
    }

    public void setDriveWheels(StaticDriveWheel[] driveWheels) {
        super.setDriveWheels(driveWheels);
        double sins = 0;
        for(StaticDriveWheel w : driveWheels){
            sins += Math.abs(w.getPowerVector()[1]);
        }
        largestPowerVector = sins;
    }
    public void superDirectDrive(double fl, double fr, double bl, double br){
        veryDirectDrive(fl, fr, bl, br);
    }

    public void directDrive(double targetAngle, double power, double headingCorrection, double pathCorrection){
        if (power != 0) {
            double headingPower = getHeadingPower(headingCorrection);
            double perpendicularPower = Range.clip(
                    getDirectionalPowerScalar(targetAngle) * (getPerpendicularPower(pathCorrection)),
                    -1 + getDriveMinimumMovementPower(), // -1
                    1-getDriveMinimumMovementPower());       // 1

            //whats happening:
            //Heading power and perp power are being set to 1 and 0 respecitvely which causes the power here to be set to 0 and then divide by 0 etc
            //Heading power is being set to 1 im assuming because the heading value is far away from the current heading, so this seems ok
            //
            //TODO WHAT's ACUTALYY HAPPENING
            //IF the heading diffence is big enough that the PID returns something greater than 0 than it clips to 1 and sucks            //is returning 0
//            BaseOpMode.addData("Heading Power", headingPower);//        1
//            BaseOpMode.addData("Perpendicalar power", perpendicularPower);//        0
//
//            BaseOpMode.addData("hypot", Math.hypot(power, perpendicularPower));
            power = Range.clip(
                    Math.hypot(power, perpendicularPower),

                    Math.min(-1 + Math.abs(headingPower), -Math.abs(perpendicularPower)), // these are both getting set to 0
                    Math.max(1-Math.abs(headingPower), Math.abs(perpendicularPower))    // this one too
            );
            double a = Math.asin(perpendicularPower / power);
            if (!Double.isNaN(a)){
                targetAngle += a;
            }
            setWheelPowers(targetAngle, power, headingPower);
        }else{
            stopDrive();
        }
    }

    public void directDrive(double targetAngle, double power, double headingCorrection){
        if (power != 0) {
            double headingPower = getHeadingPower(headingCorrection);

            power = Range.clip(power, -1 + Math.abs(headingPower), 1-Math.abs(headingPower));

            BaseOpMode.addData("Target Angle Used", 2);
            setWheelPowers(targetAngle, power, headingPower);
        }else{
            stopDrive();
        }
    }
    public void followArc(double targetAngle, double power, double headingCorrection, double pathCorrection, double r, double[] velocity) {
        targetAngle -= Location.heading();
        double headingPower = getHeadingPower(headingCorrection);
        double perpendicularPower = Range.clip(getDirectionalPowerScalar(targetAngle) * (getPerpendicularPower(pathCorrection) + getCentripetalForceConstant() * Math.pow(Math.hypot(velocity[0], velocity[1]), 2) / r), -1 + getDriveMinimumMovementPower(), 1-getDriveMinimumMovementPower());

        power = Range.clip(Math.hypot(power, perpendicularPower), Math.min(-1 + Math.abs(headingPower), -Math.abs(perpendicularPower)), Math.max(1-Math.abs(headingPower), Math.abs(perpendicularPower)));

        targetAngle += Math.asin(perpendicularPower / power);

        BaseOpMode.addData("Target Angle Used", 3);
        setWheelPowers(targetAngle, power, headingPower);
    }

    double prevDerivative = 0;
    public double[] driverOrientedStickCorrection(double[] driveStick, double timeMillis, double power) {

        double deltaY = Math.abs(yRingBuffer(Location.y()) - Location.y());
        double deltaX = Math.abs(xRingBuffer(Location.x()) - Location.x());
        if (deltaX == 0) deltaX = .00000001;

        //path[0] = x    path[1] = y
        //path[2] = x'   path[3] = y'
        //path[4] = x''  path[5] = y''

        double[] velocity = getVelocityTELE(Location.x(), Location.y(), timeMillis);
        double derivative = deltaY/deltaX;
        if(velocity[0] > .05){
            double circlePow = getCentripetalForceConstant() * Math.pow(velocity[0], 2)
                    * ((derivative-prevDerivative)/deltaX)
                    / Math.pow(Math.hypot(1, derivative), 3);
            prevDerivative = derivative;
            double rotationFactor = -Math.asin(Range.clip(circlePow / (power+.0000001), -.65, .65));
            double dX = Math.cos(rotationFactor) * driveStick[0] + Math.sin(rotationFactor) * driveStick[1];
            double dY = Math.cos(rotationFactor) * driveStick[1] - Math.sin(rotationFactor) * driveStick[0];

            return new double[]{dX, -dY};
        }
        prevDerivative = derivative;

        driveStick[1] *= -1;
        return driveStick;
    }

    private double[] getVelocityTELE (double x, double y, double time){
        int current = velocityRingBufferCount % 3;
        int last = (velocityRingBufferCount+1) % 3;

        velocityRingBuffer[current][0] = x;
        velocityRingBuffer[current][1] = y;
        velocityRingBuffer[current][3] = time;

        double dX = x-velocityRingBuffer[last][0];
        double dY = y-velocityRingBuffer[last][1];
        double dT = time-velocityRingBuffer[last][3];

        double dDistance = Math.hypot(dX, dY);

        double speed = dDistance/ dT;
        double slope;
        if(dX == 0){
            slope = 9999999;
        }else{
            slope = dY/dX;
        }

        velocityRingBufferCount++;

        return new double[]{speed, slope};
    }

    protected void setWheelPowers(double targetAngle, double power, double headingPower){
        double highestPower = 0;
        double positiveYValues = 0;
        double negativeYValues = 0;
        for(int i = 0; i < driveWheels.length; i++){
            StaticDriveWheel w = driveWheels[i];

            wheelPowers[i] = Math.cos(w.getAngle()-targetAngle);

            highestPower = Math.max(highestPower, Math.abs(wheelPowers[i]));
        }

        for(int i = 0; i < driveWheels.length; i++){
            driveWheels[i].setPower(wheelPowers[i]*power/highestPower + driveWheels[i].getRotationalConstant() * headingPower);
        }
    }
    public void setWheelPowersDirect(double fl, double fr, double bl, double br){

    }

    int xCounter = 0;
    private double xRingBuffer(double current){
        int last = (xCounter+1) % xRingBufferValues.length;

        xRingBufferValues[xCounter] = current;

        double x = xRingBufferValues[last];

        xCounter = (xCounter + 1) % xRingBufferValues.length;

        return x;
    }

    volatile int yCounter = 0;
    private double yRingBuffer(double current){
        int last = (yCounter+1) % yRingBufferValues.length;

        yRingBufferValues[yCounter] = current;

        double y = yRingBufferValues[last];

        yCounter = (yCounter + 1) % yRingBufferValues.length;

        return y;
    }



}

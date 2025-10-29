package org.firstinspires.ftc.teamcode.teamcode.KCP.Localization;


import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Hardware;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.LinearAlgebra.GeneralMatrix;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.LinearAlgebra.Vector;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class TwoWheelOdometry extends Location{

    //et variables and measure precision of instruments
    private double imuOffset = 0;
    private int imuNumber = 0;

    //location variables to be read by other classes

    //what the imu reading should be rounded to for maximum accuracy
    double IMUMaximumPrecision = 0.01;
    double IMUMaxNum;

    //defining the center of the robot to be the center of the robot
    //Values For JamieV2
    double horizontalOffset = 0.5; //vertical distance from horizontal encoder to center of robot
    double verticalOffset = 6; //
    //Values for Test Chassis
    //double verticalOffset = 7.5;
    //double horizontalOffset  7.25;
    //last vertical and horizontal encoder readings
    double hPrevDist, vPrevDist;

    //new readings
    double newVertical, newHorizontal, newHeading;
    //change in readings / calculated change
    double dVertical, dHorizontal, dHeading, dX, dY;
    //public final MotorEncoder verticalEncoder;
    //public final MotorEncoder horizontalEncoder;

    // Filter Stuff

    //public final BadBNO055 gyro;


    private DynamicComplementaryFilter guesstimator;

    private GoBildaPinpointDriver odoWheels;


    boolean isSet = false;
    double startX, startY, startHeading;
    // Tunables
    public final double alpha = 1;
    private AprilTagOdometrySource AprilODOSource;


    private double[] oldVelocity = new double[3];

    @Config
    public static class velocityFilterParameters {
        public static double alphaVelocity = 1;
    }

    ElapsedTime t;

    //intialize odometry
    public TwoWheelOdometry(double startX, double startY, double startHeading){
        super(startX, startY);
        IMUMaxNum = 1/IMUMaximumPrecision;

        odoWheels = hardware.get(GoBildaPinpointDriver.class, Hardware.odoWheels);
        odoWheels.setOffsets(175,-41.25);
        odoWheels.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odoWheels.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odoWheels.recalibrateIMU();

        isSet = false;

        this.startX = startX;
        this.startY = startY;
        this.startHeading = startHeading;

        t = new ElapsedTime();





        guesstimator = new DynamicComplementaryFilter(
                GeneralMatrix.identityMatrix(3).multiplied(alpha),
                new Vector(startX, startY, startHeading)
        );

        //get new reads on sensors (heading, Vertical encoder, and horizontal encoder)

        AprilODOSource = new AprilTagOdometrySource(startX, startY, startHeading, "blue");


        Location.heading = startHeading;


        //     gyro.setOffsetAngle(Math.toDegrees(startHeading));


    }

    @Override
    public void setCurrentHeading(double heading) {

        odoWheels.setPosition(new Pose2D(DistanceUnit.METER, 0, 0, AngleUnit.RADIANS, heading));
        Location.heading = heading;
    }

    //Uses Odo Pods
    public void localize() {
//
//        //get new reads on sensors (heading, Vertical encoder, and horizontal encoder)
//        newVertical = verticalEncoder.getPosition();
//        newHorizontal = horizontalEncoder.getPosition();
////        newHeading = gyro.getHeading();


        if (!isSet && t.seconds() >0.5) {

            isSet = true;
            odoWheels.setPosition(new Pose2D(DistanceUnit.METER, startX, startY, AngleUnit.RADIANS, startHeading));
        }
        odoWheels.update();
        //AprilODOSource.update();

        //Vector aprilTagPosition = AprilODOSource.getValidPositionVector();
        Pose2D wheelPosition = odoWheels.getPosition();
        Pose2D wheelVelocity = odoWheels.getVelocity();
        //Vector odoWheelPosition = new Vector(wheelPosition.getX(DistanceUnit.INCH), wheelPosition.getY(DistanceUnit.INCH), wheelPosition.getHeading(AngleUnit.RADIANS));
        //guesstimator.update(odoWheelPosition, aprilTagPosition);
        //newHeading = guesstimator.getAngle();

        newHeading = wheelPosition.getHeading(AngleUnit.RADIANS);


        BaseOpMode.addData("unwrappedHeading", newHeading);


        location[0] = wheelPosition.getX(DistanceUnit.METER); //guesstimator.getPosition();
        location[1] = wheelPosition.getY(DistanceUnit.METER);

        velocity[0] += velocityFilterParameters.alphaVelocity* (wheelVelocity.getX(DistanceUnit.METER) - velocity[0]);
        velocity[1] += velocityFilterParameters.alphaVelocity* (wheelVelocity.getY(DistanceUnit.METER) - velocity[1]);
        velocity[2] = wheelVelocity.getHeading(AngleUnit.RADIANS);

        Location.heading = newHeading; //guesstimator.getPosition();
        BaseOpMode.addData("wrappedHeading", newHeading);

    }

    public double getHeadingVelocity(){
        return odoWheels.getHeadingVelocity();
    }




    public double getHighestX(){
        double highestX = 0;
        if (highestX < TwoWheelOdometry.x()){
            highestX = TwoWheelOdometry.x();
        }
        return highestX;
    }
    public double getHighestY(){
        double highestY = 0;
        if (highestY < TwoWheelOdometry.y()){
            highestY = TwoWheelOdometry.y();
        }
        return highestY;
    }


    public double getVelocityY(){
        return velocity[1];
    }

    @Override
    public AprilTagProcessor getProcessor() {
        return AprilODOSource.getProcessor();
    }
}
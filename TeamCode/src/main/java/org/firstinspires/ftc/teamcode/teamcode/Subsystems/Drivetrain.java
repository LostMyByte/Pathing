// Primary Author: Mixed

package org.firstinspires.ftc.teamcode.teamcode.Subsystems;


import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode.multTelemetry;
import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Constants.focalLengthMM;
import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Constants.fx;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Drivetrain.DrivetrainDash.kD;
import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Dash.PIDTuningDash.HD;
import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Dash.PIDTuningDash.HP;
import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Dash.PIDTuningDash.rateOfChange;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains.TankDriveTrain;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Localization.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Constants;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Hardware;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Control.PID;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Vision.BallDetector;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class Drivetrain extends Subsystem {
    @Config
    public static class DrivetrainDash {
        public static double rateOfChangeThreshold = 120;
//        public static double
//                p = 0.05,
//                i = 0,
//                d = 0;
//
        public static double
            kP = 0.001,
            kI = 0,
            kD = -0.0001;
        public static double visionTurnDeadzone = 3; //silly (ignore this stuff)
        public static double visionTurn = -0.003;

        public static double visionStrafe = 0.005;
        public static double visionStrafeDeadzone = 3;
        public static double visionDrive = 0.007;
        public static double visionDriveDeadzone = 5;
        public static double visionDistanceTarget = 138; //pixels bc goofy
    }
    MecanumDrive driveWheels;

    PID pid;

    Gamepad gamepad1;
    GoBildaPinpointDriver gyro;

    boolean pid_on = false;
    boolean pid_on_last_cycle = false;
    double setPoint = 0;
    double error = 0;
    double angleRad;

    PID visionTurnPID;

    TankDriveTrain driveWheels2;

    Vector theFuckassTankDriveVector = new Vector(0,0);

    @Override
    public void update(){}
    public void update(AprilTagDetection tagNum, double x, double y, double heading, double cameraNumber){}

    @Override
    public void updateSensors() {
    }

    public void telemetry(){
        BaseOpMode.addData("Heading", gyro.getHeading());
    }

    public Drivetrain(HardwareMap hardware, double heading) {

        gamepad1 = new Gamepad();
        driveWheels2 = new TankDriveTrain();
      //  driveWheels = new MecanumDrive();
        visionTurnPID = new PID(DrivetrainDash.kP, DrivetrainDash.kI, kD);
/*
 gyro = hardware.get(GoBildaPinpointDriver.class, Hardware.odoWheels);
        if (Double.isNaN(Constants.startAngle)) {
            gyro.resetPosAndIMU();
            gyro.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, heading));
        }
        else {

            gyro.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, gyro.getHeading()- PI/2));;
        }*/
        pid = new PID(HP,0, HD);



    }


    public void stopDrive(){
        //driveWheels.veryDirectDrive(0,0,0,0);
    }


    public void nonDriverOrientedDrive (double drive, double strafe, double turn){
     //   driveWheels.veryDirectDrive((drive + strafe - turn),-(drive - strafe + turn),(drive - strafe - turn),-(drive + strafe + turn));
    }

    public void drive(double drive, double strafe, double turn, double speed, boolean lockHeading) {
        gyro.update();
        strafe = -strafe;
        //TODO DRIVER ORIENTED STUFF

        Vector2d driveVector = new Vector2d(strafe, drive);
        Vector2d rotatedVector = driveVector.rotateBy(Math.toDegrees(-gyro.getHeading()));

        BaseOpMode.addData("Heading", gyro.getHeading());

        drive = rotatedVector.getY();
        strafe = -rotatedVector.getX();
        if (!lockHeading) {
            double currentRateOfChange = gyro.getHeadingVelocity();
            if (turn != 0) {
                pid_on = false;
            } else if (currentRateOfChange <= rateOfChange) pid_on = true;

            if (pid_on && !pid_on_last_cycle) {
                setPoint = gyro.getHeading();
            } else if (pid_on) {
                turn = pid.getCorrectionHeading(gyro.getHeading(), setPoint);
            }
            pid_on_last_cycle = pid_on;
            BaseOpMode.addData("Rate Of Change", currentRateOfChange);
            BaseOpMode.addData("Actual Heading", gyro.getHeading());
            BaseOpMode.addData("SetPoint", setPoint);
        } else {
            turn = pid.getCorrectionHeading(gyro.getHeading(),turn);
        }
        pid.setConstants(HP, 0, HD);
            //SET POINT JUST INCREASES constantly
//        BaseOpMode.addData("targetHeading", setPoint);
//        BaseOpMode.addData("pidOn", pid_on);
//        BaseOpMode.addData("current roc", currentRateOfChange);


//        updatePID();


/*
        motorfl.setPower(-(drive - strafe + turn) * speed);
        motorfr.setPower((drive + strafe - turn) * speed);
        motorbl.setPower(-(drive + strafe + turn) * speed);
        motorbr.setPower((drive - strafe - turn) * speed);

         */

      //  driveWheels.veryDirectDrive((drive + strafe - turn) * speed,-(drive - strafe + turn) * speed,(drive - strafe - turn) * speed,-(drive + strafe + turn) * speed);
    }


    public void updatePID(){
        pid.setConstants(0,0,0);
    }

    public double getHeading(){
        return gyro.getHeading();
    }

    public void setTargetHeading(double heading){
        setPoint = heading;
    }

    public void ballFollow(int targetWidth){

        //Rect rectangle = TestPipelineBlue.getRectangle();

        double drive = 0;

      /*
  if ((rectangle.height)/2 < 120){
            //drive up to box
            drive = 1;
        }*/



       // double strafe = 0; //this ain't meccanum
        double turn = 0 ;


//this should correct for the x coordinate

        if(Math.abs(BallDetector.getError(false))> DrivetrainDash.visionTurnDeadzone && BallDetector.targetDetected&&BallDetector.getWidth(false)>30){

            //error 199
    visionTurnPID.setConstants(DrivetrainDash.kP, DrivetrainDash.kI, kD);

            //this code is now 3 yrs old (made in 2023) and I still havn't made anything more useful/reusable

            turn =
            // BallDetector.getError(false)* DrivetrainDash.kP

        -visionTurnPID.getCorrection(BallDetector.getError(false));

        } else{
           // multTelemetry.addData("Status","not moving");
            turn = 0;
         //   strafe = 0;
        }

        double distanceError = scuffedDistance(BallDetector.getWidth(false), targetWidth);
        //double distanceError = distance(BallDetector.getWidth(false)) - DrivetrainDash.visionDistanceTarget;
        if(Math.abs(distanceError) > DrivetrainDash.visionDriveDeadzone && BallDetector.targetDetected&&BallDetector.getWidth(false)>30){
            drive = distanceError * DrivetrainDash.visionDrive+0.001;
        }else{
            drive = 0;
        }



        multTelemetry.addData("Error", BallDetector.getError(false));
      //  multTelemetry.addData("distance error", distanceError);
        multTelemetry.addData("turn", turn );
        multTelemetry.addData("drive", drive);
        multTelemetry.addData("Width", BallDetector.getWidth(false));
       // multTelemetry.addData("distance", distance(BallDetector.getWidth(false)));
        multTelemetry.addData("angle in radians", angleRad);

        driveWheels2.drive(drive,-turn);
      //  driveWheels.veryDirectDrive(drive +strafe -turn,drive -strafe +turn,drive -strafe -turn,drive +strafe +turn);

/* fl.setPower((drive -strafe +turn));
       fr.setPower((drive +strafe -turn));
        bl.setPower((drive +strafe +turn));
        br.setPower((drive -strafe -turn));*/

    }
    public double distance(double widthPixels){
        //double angleDeg = ((120*widthPixels)/320) /2;
       // angleRad = angleDeg * (PI/180);
        double diameterOfObject = 12.7/100; //in meters
        double distance;
        distance = diameterOfObject*fx/widthPixels-focalLengthMM;
        //distance = pixelsToMeters*widthPixels; //this will never work, but it's a neat idea
        //not real yet, ran out of time
        //real diameter times focal length in px over pixel diameter minus focal length MM
        //distance = Math.sqrt(Math.pow(30/Math.tan(angleRad),2)-(Math.pow(height,2)));
        return distance;
    }

    public int scuffedDistance(int widthPixels, int targetDistance){
       int distance;
           distance = targetDistance-widthPixels;

        return distance;
        //gives dist in pixels, trust
    }

    public void resetHeading(){
        gyro.resetPosAndIMU();
    }

}

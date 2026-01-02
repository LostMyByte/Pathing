// Primary Author: Mixed

package org.firstinspires.ftc.teamcode.teamcode.Subsystems;


import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode.multTelemetry;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.TeliOpDrivetrain.DrivetrainDash.kDturn;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.TeliOpDrivetrain.DrivetrainDash.visionTurn;
import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Constants.focalLengthMM;
import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Constants.fx;
import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Dash.PIDTuningDash.HD;
import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Dash.PIDTuningDash.HP;
import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Dash.PIDTuningDash.Kdh;
import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Dash.PIDTuningDash.Kph;
import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Dash.PIDTuningDash.rateOfChange;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains.TankDriveTrain;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Localization.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Localization.Location;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Constants;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Hardware;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Control.PID;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Dash.PIDTuningDash;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Vision.BallChaser;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class TeliOpDrivetrain extends TankDriveTrain {
    @Config
    public static class DrivetrainDash {
        public static double rateOfChangeThreshold = 120;
        //        public static double-
//                p = 0.05,
//                i = 0,
//                d = 0;
//
        public static double
                kPturn = 0.001,
                kIturn = 0,
                kDturn = 0.005,
                kPdrive = 0.007,
                kIdrive = 0,
                kDdrive = -0.005;

        public static double visionTurnDeadzone = 3; //silly (ignore this stuff)
        public static double visionTurn = 0.0001;

        public static double visionStrafe = 0.005;
        public static double visionStrafeDeadzone = 3;
        public static double visionDrive = 0.001;
        public static double visionDriveDeadzone = 5;
        public static double visionDistanceTarget = 138; //pixels bc goofy
    }


    PID pid;

    Gamepad gamepad1;
    GoBildaPinpointDriver gyro;

    boolean pid_on = false;
    boolean pid_on_last_cycle = false;
    double setPoint = 0;
    double error = 0;
    double angleRad;

    PID visionTurnPID;
    PID visionDrivePID;

    Vector theFuckassTankDriveVector = new Vector(0, 0, 0);

    @Override
    public void update() {
    }

    public void update(AprilTagDetection tagNum, double x, double y, double heading, double cameraNumber) {
    }

    @Override
    public void updateSensors() {
    }

    public void telemetry() {
        BaseOpMode.addData("Heading", gyro.getHeading());
    }

    public TeliOpDrivetrain(HardwareMap hardware, double heading) {

        gamepad1 = new Gamepad();
        //  driveWheels = new MecanumDrive();
        visionTurnPID = new PID(DrivetrainDash.kPturn, DrivetrainDash.kIturn, kDturn);
        visionDrivePID = new PID(DrivetrainDash.kPdrive, DrivetrainDash.kIdrive, DrivetrainDash.kDdrive);


        pid = new PID(HP, 0, HD);
        drivePID = new PID(0, 0, 0);


    }


    public void stopDrive() {
        //driveWheels.veryDirectDrive(0,0,0,0);
    }


    public void nonDriverOrientedDrive(double drive, double strafe, double turn) {
        //   driveWheels.veryDirectDrive((drive + strafe - turn),-(drive - strafe + turn),(drive - strafe - turn),-(drive + strafe + turn));
    }

    public void drive(double drive, double strafe, double turn, double speed, boolean lockHeading) {
        strafe = -strafe;
        //TODO DRIVER ORIENTED STUFF

        Vector2d driveVector = new Vector2d(strafe, drive);
        Vector2d rotatedVector = driveVector.rotateBy(Math.toDegrees(loc.getPosH()));

        BaseOpMode.addData("Heading", loc.getPosH());

        drive = rotatedVector.getY();
        strafe = -rotatedVector.getX();
        if (!lockHeading) {
            double currentRateOfChange = loc.getVelH();
            if (turn != 0) {
                pid_on = false;
            } else if (currentRateOfChange <= rateOfChange) pid_on = true;

            if (pid_on && !pid_on_last_cycle) {
                setPoint = loc.getPosH();
            } else if (pid_on) {
                turn = pid.getCorrectionHeading(loc.getPosH(), setPoint);
            }
            pid_on_last_cycle = pid_on;
            BaseOpMode.addData("Rate Of Change", currentRateOfChange);
            BaseOpMode.addData("Actual Heading", loc.getPosH());
            BaseOpMode.addData("SetPoint", setPoint);
        } else {
            turn = pid.getCorrectionHeading(loc.getPosH(), turn);
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


    public void updatePID() {
        pid.setConstants(0, 0, 0);
    }

    public double getHeading() {
        return gyro.getHeading();
    }

    public void setTargetHeading(double heading) {
        setPoint = heading;
    }

    public void ballFollow(int visionDistanceTarget, boolean trueIfGreen) {

        //Rect rectangle = TestPipelineBlue.getRectangle();

        double drive = 0;

      /*
  if ((rectangle.height)/2 < 120){
            //drive up to box
            drive = 1;
        }*/

        double aspectRatio = ((double) (BallChaser.getWidth(trueIfGreen)) / (BallChaser.getHeight(trueIfGreen)));

        // double strafe = 0; //this ain't meccanum
        double turn = 0;


//this should correct for the x coordinate

        if (Math.abs(BallChaser.getError(trueIfGreen)) > DrivetrainDash.visionTurnDeadzone && BallChaser.targetDetected && BallChaser.getWidth(trueIfGreen) > 30) {
            //error 199
            visionTurnPID.setFeedForward(visionTurn);
            //error 199
            visionTurnPID.setConstants(DrivetrainDash.kPturn, DrivetrainDash.kIturn, kDturn);

            //this code is now 3 yrs old (made in 2023) and I still havn't made anything more useful/reusable

            turn =
                    // BallDetector.getError(false)* DrivetrainDash.kP

                    -visionTurnPID.getCorrection(BallChaser.getError(trueIfGreen));

        } else {
            // multTelemetry.addData("Status","not moving");
            turn = 0;
            //   strafe = 0;
        }
        double distanceError = scuffedDistance(BallChaser.getWidth(trueIfGreen), visionDistanceTarget, trueIfGreen);
        //double distanceError = distance(BallDetector.getWidth(false)) - DrivetrainDash.visionDistanceTarget;
        if (Math.abs(distanceError) > DrivetrainDash.visionDriveDeadzone && BallChaser.targetDetected && BallChaser.getWidth(trueIfGreen) > 30) {
            visionDrivePID.setFeedForward(DrivetrainDash.visionDrive);
            drive = visionDrivePID.getCorrection(distanceError);
            //distanceError * DrivetrainDash.visionDrive+0.001;
        } else {
            drive = 0;
        }


        multTelemetry.addData("Error", BallChaser.getError(trueIfGreen));
        //  multTelemetry.addData("distance error", distanceError);
        multTelemetry.addData("turn", turn);
        multTelemetry.addData("drive", drive);
        multTelemetry.addData("Width", BallChaser.getWidth(trueIfGreen));
        multTelemetry.addData("height", BallChaser.getHeight(trueIfGreen));
        // multTelemetry.addData("distance", distance(BallDetector.getWidth(false)));
        multTelemetry.addData("angle in radians", angleRad);
        multTelemetry.addData("scuffed distance", scuffedDistance(BallChaser.getWidth(trueIfGreen), visionDistanceTarget, trueIfGreen));
        multTelemetry.addData("aspect ratio", aspectRatio);
        drive(drive, turn);
        //  driveWheels.veryDirectDrive(drive +strafe -turn,drive -strafe +turn,drive -strafe -turn,drive +strafe +turn);
//if check aspect ratio of detection, then multiply target width by ratio
/* fl.setPower((drive -strafe +turn));
       fr.setPower((drive +strafe -turn));
        bl.setPower((drive +strafe +turn));
        br.setPower((drive -strafe -turn));*/

    }

    public double distance(double widthPixels) {
        //double angleDeg = ((120*widthPixels)/320) /2;
        // angleRad = angleDeg * (PI/180);
        double diameterOfObject = 12.7 / 100; //in meters
        double distance;
        distance = diameterOfObject * fx / widthPixels - focalLengthMM;
        //distance = pixelsToMeters*widthPixels; //this will never work, but it's a neat idea
        //not real yet, ran out of time
        //real diameter times focal length in px over pixel diameter minus focal length MM
        //distance = Math.sqrt(Math.pow(30/Math.tan(angleRad),2)-(Math.pow(height,2)));
        return distance;
    }

    public double scuffedDistance(int widthPixels, int targetDistance, boolean trueIfGreen) {

        double aspectRatio = ((double) (BallChaser.getWidth(trueIfGreen)) / (BallChaser.getHeight(trueIfGreen)));
        double distance;
        distance = (targetDistance * aspectRatio) - widthPixels;

        return distance;
        //gives dist in pixels, trust
    }

    public void resetHeading() {
        gyro.resetPosAndIMU();
    }

    public void PIDdrive(double drive, double turn) {

        double currentRateOfChange = gyro.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
        if (turn != 0) {
            pid_on = false;
        } else if (currentRateOfChange <= rateOfChange) {
            pid_on = true;
        }

        if (pid_on && !pid_on_last_cycle) {

            setPoint = gyro.getHeading(AngleUnit.RADIANS);

        } else if (pid_on) {
            turn = pid.getCorrectionHeading(gyro.getHeading(AngleUnit.RADIANS), setPoint);
        }
        pid_on_last_cycle = pid_on;
        BaseOpMode.addData("Rate Of Change", currentRateOfChange);
        BaseOpMode.addData("Actual Heading", gyro.getHeading(AngleUnit.RADIANS));
        BaseOpMode.addData("SetPoint", setPoint);

        pid.setConstants(HP, 0, HD);
        move(new Vector(-drive, -turn));
    }

    public void PIDdrive(double drive, double turn, double power) {

        double currentRateOfChange = loc.getVelH();
        if (turn != 0) {
            pid_on = false;
        } else if (Math.abs(currentRateOfChange) <= rateOfChange) {
            pid_on = true;
        }

        if (pid_on && !pid_on_last_cycle) {

            setPoint = loc.getPosH();

        } else if (pid_on) {
            //turn = pid.getCorrection(loc.getPosH(), setPoint);
        }
        pid_on_last_cycle = pid_on;
        BaseOpMode.addData("Actual Heading", loc.getPosH());
        BaseOpMode.addData("SetPoint", setPoint);

        pid.setConstants(HP, 0, HD);
        drive = drive * power;
        turn = turn * power;
        move(new Vector(-drive, -turn));
    }

    double x = 0;
    double y = 0;
    double h = 0;
    double startX;
    double startY;
    PID drivePID;

    public void updateOdo(double x, double y, double h) {
        this.x = x;
        this.y = y;
        this.h = h;
    }

    //takes in how much you want to drive. Do not try to turn and drive simultaneously it WILL SUCK
    public void driveDumb(double targDist, double targHeading, double power, boolean newPath) {
        //first loop running the path newPath == true, then set to false
        if (newPath) {
            startX = x;
            startY = y;
        }
        double deltaX = x - startX;
        double deltaY = y - startY;
        double distanceTraveled = Math.sqrt(deltaX * deltaX + deltaY * deltaY) * Math.signum(targDist);

        drivePID.setConstants(PIDTuningDash.Kpd, 0, PIDTuningDash.Kdd);
        drivePID.setLowerLimit(PIDTuningDash.Kfd);
        drivePID.setDeadZone(2);

        pid.setConstants(HP, 0, HD);
        pid.setLowerLimit(PIDTuningDash.HF);
        pid.setDeadZone(.1);

        double drive = drivePID.getCorrection(targDist - distanceTraveled) * power;
        double turn = pid.getCorrection(h - targHeading);

        move(new Vector(-drive, -turn));

        BaseOpMode.addData("deltaX", deltaX);
        BaseOpMode.addData("deltaY", deltaY);
        BaseOpMode.addData("distanceTraveled", distanceTraveled);
        BaseOpMode.addData("targDist", targDist);
        BaseOpMode.addData("drive", drive);
        BaseOpMode.addData("h", h);
        BaseOpMode.addData("targh", targHeading);
    }

    public void dumbDriveToPos(double x, double y, double power) {
        double targetDistance = new Vector(x-loc.getPosX(), y-loc.getPosY()).magnitude();
        double targetHeading = Math.atan2(y - loc.getPosY(), x-loc.getPosX()) + Math.PI/2;
        if (Math.abs(loc.getPosH() - targetHeading) > 0.3) driveDumb(0, targetHeading, power, true);
        else driveDumb(targetDistance, targetHeading, power, true);
    }
}

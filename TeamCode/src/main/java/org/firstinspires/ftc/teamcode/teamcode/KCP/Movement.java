package org.firstinspires.ftc.teamcode.teamcode.KCP;


import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Dash.PIDTuningDash.DeadZone;
//import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Dash.PIDTuningDash.Kdd;
import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Dash.PIDTuningDash.HD;
import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Dash.PIDTuningDash.HP;
import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Dash.PIDTuningDash.Kdd;
import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Dash.PIDTuningDash.Kdh;
//import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Dash.PIDTuningDash.Kds;
//import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Dash.PIDTuningDash.Kfd;
//import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Dash.PIDTuningDash.Kfs;
import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Dash.PIDTuningDash.Kds;
import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Dash.PIDTuningDash.Kfd;
import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Dash.PIDTuningDash.Kfs;
import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Dash.PIDTuningDash.Kih;
//import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Dash.PIDTuningDash.Kpd;
import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Dash.PIDTuningDash.Kpd;
import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Dash.PIDTuningDash.Kph;
import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Dash.PIDTuningDash.Kps;
import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Dash.PIDTuningDash.rateOfChange;
//import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Dash.PIDTuningDash.Kps;
import static java.lang.Math.abs;
import static java.lang.Math.signum;

import org.firstinspires.ftc.teamcode.teamcode.Utilities.Control.MotionProfiler;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Dash.PIDTuningDash;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.zLibraries.Utilities.PIDController;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.zLibraries.Utilities.Vector2d;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.KCP.DriveClasses.AbstractClasses.DriveTrain;
import org.firstinspires.ftc.teamcode.teamcode.KCP.DriveClasses.MecanumDrive;
import org.firstinspires.ftc.teamcode.teamcode.KCP.Localization.Location;
import org.firstinspires.ftc.teamcode.teamcode.KCP.Localization.TwoWheelOdometry;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Control.PID;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class Movement extends Subsystem {
    /**
     * In this case odometry, but any extension of the localization class can be passed in
     */
    private final Location odo;
    PID pidHeading;
    PIDController pfdDrive;
    PIDController pfdStrafe;

    PID pidHeadingTeleOp;

    double error;
    double fr;
    double fl;
    double bl;
    double br;

    double lastErrorHeading;
    double releaseAngle;
    double targetAngle;
    double lastErrorDrive;
    double lastErrorStrafe;
    double inputTurn;

    double holdX;
    double holdY;
    double holdH;
    int holdNum;

    double integral;


    MotionProfiler driveProfile;
    MotionProfiler strafeProfile;

    boolean pid_on = false;
    boolean pid_on_last_cycle = false;
    double setPoint = 0;
    double errorH = 0;


    /**
     * Any type of drivebase that extends the drivetrain class
     */
    public final DriveTrain drive;
    private final ElapsedTime runtime;
    int counter;


    public enum CorrectionMethods {
        PID,
        Profiled
    }

    CorrectionMethods activeCorrection = CorrectionMethods.PID;

    /**
     * Delcare drivebase and localization method
     * @param startX
     * @param startY
     */
    public Movement(double startX, double startY, double heading){
        drive = new MecanumDrive();


        drive.lockDrive();
        drive.stopDrive();

        //odo = new TwoWheelOdometry(startX, startY, heading);
        odo = new TwoWheelOdometry(startX, startY, heading);


        runtime = new ElapsedTime();
        runtime.reset();
        pidHeading = new PID(Kph,Kih,Kdh);
        pfdDrive = new PIDController(Kpd,0,Kdd,0);
        pfdDrive.setFComponent(Kfd);
        pfdStrafe = new PIDController(Kps,0,Kpd,0);
        pfdStrafe.setFComponent(Kfs);


        driveProfile = new MotionProfiler(
                PIDTuningDash.kVd,
                PIDTuningDash.Kld,
                PIDTuningDash.velPd,
                PIDTuningDash.acceld,
                PIDTuningDash.maxVd,
                PIDTuningDash.deceld,
                PIDTuningDash.deceldClose);

        driveProfile.setTitle("Drive");
        //driveProfile.setLogging(true);

        strafeProfile = new MotionProfiler(
                PIDTuningDash.kVs,
                PIDTuningDash.Kls,
                PIDTuningDash.velPs,
                PIDTuningDash.accels,
                PIDTuningDash.maxVs,
                PIDTuningDash.decels,
                PIDTuningDash.decelsClose);

        strafeProfile.setTitle("Strafe");
        pidHeadingTeleOp = new PID(HP,0, HD);
        //strafeProfile.setLogging(true);


    }





    /**
     * Needs to be called every loop
     */
    public void update(){
        /* driveProfile.setConstants(
                PIDTuningDash.kVd,
                PIDTuningDash.Kld,
                PIDTuningDash.velPd,
                PIDTuningDash.acceld,
                PIDTuningDash.maxVd,
                PIDTuningDash.deceld,
                PIDTuningDash.deceldClose);

        strafeProfile.setConstants(
                PIDTuningDash.kVs,
                PIDTuningDash.Kls,
                PIDTuningDash.velPs,
                PIDTuningDash.accels,
                PIDTuningDash.maxVs,
                PIDTuningDash.decels,
                PIDTuningDash.decelsClose); */
    }

    public void update(AprilTagDetection aprilTags, double x, double y, double heading, double cameraNumber){
        BaseOpMode.addData("Movement Updated", "");

        odo.update();
    }



    @Override
    public void updateSensors() {

    }

    boolean velocityWasHigh = false;
    int stoppedCount = 0;



    /**
     * Robot should always be trying to hold a position in autonomous
     //* @param x - x position to hold
     //     * @param y - y position to hold
     //     * @param h - heading to hold
     //     * @param hF - heading factor for determining when stopped
     //     * @param mF - movement factor for determining when stopped
     //     * @return - if robot is settled
     */

    public double pidHeading(double target, double kp, double ki, double kd, double current) {
        double error;
        if (Double.isNaN(current)) {
            error = lastErrorHeading;
        }
        else {
            error = target - current;
        }
        integral += error;
        double derivative = error - lastErrorHeading;

        if (error > 180) {
            error -= 360;
        } else if (error < -180) {
            error += 360;
        }
        BaseOpMode.addData("error", error);
        double correction = (error * kp) + (integral * ki) + (derivative * kd);
        lastErrorHeading = error;
//        multTelemetry.addData("target", target);
//        multTelemetry.addData("current", current);
//        multTelemetry.addData("error", error);
        return correction;
    }

    public double pfdDrive(double kp, double kd, double kf, double error) {
        double derivative = error;
        double correction = (error * kp) + (derivative * kd);
        if (abs(error) > DeadZone) {
            correction += signum(error) * kf;
        }
        return correction;
    }



    public void setActiveCorrectionMethod(CorrectionMethods correction) {
        this.activeCorrection = correction;
    }

    public void goToPos(double targetX, double targetY, double targetHeading, double currentX, double currentY, double currentHeading, double speed) {

        double[] velocity = getVelocity();

        Vector2d driveVector = new Vector2d(targetX - currentX, targetY - currentY);
        Vector2d velocityVector = new Vector2d(velocity[0], velocity[1]);
        Vector2d rotatedVector = driveVector.rotate(-Math.toRadians(currentHeading));
        Vector2d rotatedVelocityVector = velocityVector.rotate(-Math.toRadians(currentHeading));
        BaseOpMode.addData("heading", currentHeading);
        BaseOpMode.addData("targHeading", Math.toRadians(targetHeading));

        inputTurn = pidHeading(targetHeading, Kph, Kih, Kdh, currentHeading);

        BaseOpMode.addData("correction", inputTurn);
        double driveCorrection = 0;
        double strafeCorrection = 0;
        switch (activeCorrection) {
            case PID:
                 driveCorrection= -pfdDrive(Kpd, Kdd, Kfd, rotatedVector.y);
                 strafeCorrection = -pfdDrive(Kps, Kds, Kfs, rotatedVector.x);
                 break;
            case Profiled:
                driveCorrection = driveProfile.getCorrection(rotatedVector.y, rotatedVelocityVector.y);
                strafeCorrection = strafeProfile.getCorrection(rotatedVector.x, rotatedVelocityVector.x);
                break;

        }

        fr = ((driveCorrection - strafeCorrection - inputTurn) * speed);
        fl = ((driveCorrection + strafeCorrection + inputTurn) * speed);
        br = ((driveCorrection + strafeCorrection - inputTurn) * speed);
        bl = ((driveCorrection - strafeCorrection + inputTurn) * speed);
        drive.veryDirectDrive(fr,fl,br,bl);
    }
    public boolean hold(double xCoordinate, double yCoordinate, double heading, double speed) {
        if (Location.x() != xCoordinate || Location.y() != yCoordinate || Location.heading() != heading) {
            goToPos(xCoordinate, yCoordinate, Math.toDegrees(heading), Location.x(), Location.y(), Math.toDegrees(Location.heading()), speed);
            return true;
        } else {
            return false;
        }
    }


    public boolean holdPosition(double x, double y, double h) {
        hold(x,y,h,1);
        if ((y > (Location.y() -.6) && (y < (Location.y() +.6) && (h > (Location.heading() -.6) && (h < (Location.x() +.6)))))) {
            return true;
        } else {
            return false;
        }
    }
    public boolean holdPosition(double x, double y, double h, double speed){

        hold(x, y, h, speed);
        if ((y > (Location.y() -.6) && (y < (Location.y() +.6) && (h > (Location.heading() -.6) && (h < (Location.x() +.6)))))) {
            return true;
        } else {
            return false;
        }
    }

    public boolean holdPosition(double x, double y, double h, double speed, double completionThreshold){
        hold(x, y, h, speed);
        if ((y > (Location.y() - completionThreshold) && (y < (Location.y() + completionThreshold) && (h > (Location.heading() - completionThreshold) && (h < (Location.x() + completionThreshold)))))) {
            return true;
        } else {
            return false;
        }
    }
//    && (x < (Location.x() +.6) && (y > (Location.y() -.6) && (y < (Location.y() +.6) && (h > (Location.heading() -.6) && (h < (Location.x() +.6)))))



    public void drive(double driveM, double strafeM, double turn, double speed, boolean lockHeading) {

        strafeM = -strafeM;
        //TODO DRIVER ORIENTED STUFF

        com.arcrobotics.ftclib.geometry.Vector2d driveVector = new com.arcrobotics.ftclib.geometry.Vector2d(strafeM, driveM);
        com.arcrobotics.ftclib.geometry.Vector2d rotatedVector = driveVector.rotateBy(Math.toDegrees(getRawHeading()));

        BaseOpMode.addData("Heading", getRawHeading());

        driveM = rotatedVector.getY();
        strafeM = -rotatedVector.getX();
        if (!lockHeading) {
            double currentRateOfChange = getVelocity()[2];
            if (turn != 0) {
                pid_on = false;
            } else if (currentRateOfChange <= rateOfChange) pid_on = true;

            if (pid_on && !pid_on_last_cycle) {
                setPoint = getRawHeading();
            } else if (pid_on) {
                turn = pidHeadingTeleOp.getCorrectionHeading(getRawHeading(), setPoint);
            }
            pid_on_last_cycle = pid_on;
            BaseOpMode.addData("Rate Of Change", currentRateOfChange);
            BaseOpMode.addData("Actual Heading", getRawHeading());
            BaseOpMode.addData("SetPoint", setPoint);
        } else {
            turn = pidHeadingTeleOp.getCorrectionHeading(getRawHeading(),turn);
        }
        pidHeadingTeleOp.setConstants(HP, 0, HD);
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
        drive.veryDirectDrive((driveM - strafeM + turn) * speed,(driveM + strafeM - turn) * speed,(driveM + strafeM + turn) * speed,(driveM - strafeM - turn) * speed);
    }

    public void shiftLocation(double x, double y){
        Location.shiftLocation(x,y);
    }

    public void stopDrive(){
        drive.stopDrive();
    }

    public void driveBlind(double x, double heading){
        holdPosition(x,Location.y() + 2, heading);
    }

    public void setPosition(double x, double y){
        odo.setPosition(x, y);
    }

    public double getX(){
        return odo.x();
    }
    public double getY(){
        return odo.y();
    }
    public double getHeading(){
        return odo.heading();
    }

    public double getRawHeading() {
        return odo.headingUnwrapped();
    }

    public double[] getVelocity() {
        return odo.getVelocity();
    }

    public void setHeading(double heading){
        odo.setCurrentHeading(heading);
    }

    public void driveBlind(double power){
        drive.veryDirectDrive(-power, -power, -power, -power);
    }

}

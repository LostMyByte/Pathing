package org.firstinspires.ftc.teamcode.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Constants.B;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Constants.Team.BLUE;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Constants.Team.RED;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Constants.goalAprilTagHeight;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Constants.limelightAngleOffset;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Constants.limelightLensHeightFromGround;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Constants.team;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.TeliOp.TestOpModes.ShooterTest;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Control.PID;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Control.RingBuffer;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Dash.DashPositions;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Dash.PIDTuningDash;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Dash.ShooterDash;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.HardwareDevices.Motor;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.HardwareDevices.Servo;

import java.util.ArrayList;
import java.util.List;

public class Shooter extends Subsystem{

    Motor shooter1;
    Motor shooter2;
    Motor turret;
    Servos.Hood hood;
    AnalogInput turretEncoder;
    double turretAngle;
    PID shooterPDF;
    PID turretPDL;
    double targetShooterRPM;
    double xPosition;
    double yPosition;
    double xVelocity;
    double yVelocity;
    double heading;
    ShooterStates shooterState;
    double ticksPerRotation;
    double turretStartAngle;
    double turretTargetAngle;
    Servo readyToShootIndicator;
    Servo angleWrapWarningLight;

    Limelight3A limelight;

    boolean hoodCanShoot = false;
    boolean turretCanShoot = false;
    double turretMarginForError = .15;
    double turretMaxRotation;
    ElapsedTime warningLightTimer;
    BallColors[] pattern;
    Constants.Team team;

    ArrayList<BallColors> currentRamp;

    double degreesYtoApriltag;
    double radsYtoApriltag;
    double distanceAway;


    public Shooter(HardwareMap hardwareMap, double turretStartAngle, Constants.Team team){
        //Things are commented to prepare for the first tests of the shooter where we will only have the flywheel.

        shooter1 = new Motor(Hardware.shooter1);
        shooter2 = new Motor(Hardware.shooter2);
        hood = new Servos.Hood();
        //turret = new Motor(Hardware.turret);
        //turretEncoder = hardwareMap.get(AnalogInput.class, "turretEncoder");
        shooterPDF = new PID(0,0,0);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // make number higher to get more data
        limelight.pipelineSwitch(1);
        limelight.start();

        limelight.reloadPipeline();
        this.team = team;

        //pattern = new BallColors[3];
        shooterState = ShooterStates.OBELISK;
        //currentRamp = new ArrayList<BallColors>();
        //this.turretStartAngle = turretStartAngle;

        //readyToShootIndicator = new Servo(Hardware.indicatorLight);
        //angleWrapWarningLight = new Servo(Hardware.angleWrapWarningLight);
        //warningLightTimer = new ElapsedTime();

    }

    public void work(){
        switch (shooterState){
            case ACTIVE:
                aim();
                updateShooter();
                break;
            case NOTACTIVE:
                aim();
                break;
            case OBELISK:
                break;
            case SHOOTERTESTING:
                updateTargeting();
                break;
        }
    };



    public void updateShooter(){
        shooterPDF.setConstants(PIDTuningDash.ShooterP,0,PIDTuningDash.ShooterD);
        //sets the feedforward to the voltage needed to hold the target velocity. Values ob
        if (targetShooterRPM > 0){
        shooterPDF.setFeedForward(((targetShooterRPM * 0.00318451)+1.48267)/12);
        } else {
            shooterPDF.setFeedForward(0);
        }

        double correction = shooterPDF.getCorrection(getShooterRPM(),targetShooterRPM);




        shooter1.setPower(correction);
        shooter2.setPower(-correction);
    }

    public void getPattern(){
        int id = 0;
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            id = fiducial.getFiducialId();
        }
        if(id == 21){
            pattern[0] = BallColors.GREEN;
            pattern[1] = BallColors.PURPLE;
            pattern[2] = BallColors.PURPLE;
        }
        else if(id == 22){
            pattern[0] = BallColors.PURPLE;
            pattern[1] = BallColors.GREEN;
            pattern[2] = BallColors.PURPLE;
        }
        else if(id == 23){
            pattern[0] = BallColors.PURPLE;
            pattern[1] = BallColors.PURPLE;
            pattern[2] = BallColors.GREEN;
        }
    }

    public void updateTurret(){
        double currentAngle = turret.encoder.getPosition();
        //convert this to radians and wrap the angle
        currentAngle = currentAngle * (2*Math.PI/ticksPerRotation);

        //DONT angle wrap because the wiring means we can't actually spin around multiple times

        turretPDL.setConstants(0,0,0);
        turretPDL.getCorrectionHeading(currentAngle,turretTargetAngle);


        turret.setPower(turretPDL.getCorrectionHeading(currentAngle,turretTargetAngle));

        if (Math.abs(currentAngle-turretTargetAngle) < turretMarginForError){
            turretCanShoot = true;
        } else {
            turretCanShoot = false;
        }
    }

    double filteredRPM = 0;
    double rpmAlpha = 0.2;
    public double getShooterRPM(){
        double rpm = (Math.abs(shooter1.getVelocity()/ShooterTest.ShooterDash.ticksPerRotation*60)+Math.abs(shooter2.getVelocity()/ShooterTest.ShooterDash.ticksPerRotation*60))/2;
        filteredRPM = alpha*(rpm) + (1-rpmAlpha) * filteredRPM;
        return filteredRPM;
    }

    public double getTargetShooterRPM(){
        return targetShooterRPM;
    }
    public double setTargetBallSpeed(double speed){
        //make a regression comparing ball exit velocity to shooter RPM
        targetShooterRPM = (speed - 1.20826)/0.00263999;
        return targetShooterRPM;
    }
    public void setTargetShooterRPM(double RPM){
        targetShooterRPM = RPM;
    }

    public void aim(){

        hood.setPositionInterpolated(getHoodAngle());
        /*
        turretTargetAngle = Math.asin(xVelocity/getBallSpeed()) - heading;//sets target angle to face goal. does this by setting target angle the negitive heading (current difference in degrees from target) and also accounts for fact that robot moves


        while (turretTargetAngle > turretMaxRotation){
            turretTargetAngle -= 2*Math.PI;
        }
        while (turretTargetAngle < 0){
            turretTargetAngle += 2*Math.PI;
        }
        updateTurret();

        if (canRobotShoot()){
            readyToShootIndicator.setPosition(0.5);
        }
        else {
            readyToShootIndicator.setPosition(0.28);
        }

        if (Math.abs(turretTargetAngle - turretMaxRotation) < 0.3){
            if (warningLightTimer.seconds() > .5){
                warningLightTimer.reset();
            }
            else if (warningLightTimer.seconds() < 0.25){
                angleWrapWarningLight.setPosition(0.388);
            } else {
                angleWrapWarningLight.setPosition(0);
            }
        } else {
            angleWrapWarningLight.setPosition(0);
        }
        */

    }


    private  double actualDistance = 0;
    private double alpha = 0.2;
    public void updateTargeting(){
        int id = 0;
        double tx = 0;
        double ty = 0;
        double distance = 0;
        double yaw = 0;
        double pitch = 0;
        double roll = 0;
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            id = fiducial.getFiducialId();// The ID number of the fiducial
            double degreesYtoApriltag = fiducial.getTargetYDegrees() + limelightAngleOffset; //gets angle to limelight along x plane
            double radsYtoApriltag = degreesYtoApriltag * (Math.PI/180);
            distance = (goalAprilTagHeight - limelightLensHeightFromGround)/Math.tan(radsYtoApriltag);
            //values obtained from regression because the cameras braindead
            //distance = distance + 0.0764839 * Math.sin(2.06539 * distance - 2.68937) + 0.158071;
            ty = limelight.getLatestResult().getTy(); // gets degrees to crosshair from primary target along y axis
            tx = limelight.getLatestResult().getTx();// gets degrees to crosshair from primary target along x axis
            yaw = fiducial.getCameraPoseTargetSpace().getOrientation().getPitch(AngleUnit.DEGREES) + 90;
            if (yaw < 93){
                yaw += 87;
            }
            else if (yaw > 93){
                yaw += 3;
            } else { yaw += 90;}

            yaw = Math.toRadians(yaw);
          //  roll = fiducial.getCameraPoseTargetSpace().getOrientation().getRoll(AngleUnit.DEGREES);
           // yaw = fiducial.getCameraPoseTargetSpace().getOrientation().getYaw(AngleUnit.DEGREES);
//roll is pitch, pitch is roll and yaw is roll
        }
        if (!Double.isNaN(yaw)){
        BaseOpMode.addData("rawDistance", distance);
        BaseOpMode.addData("angle?", yaw);
      //  BaseOpMode.addData("pitch", pitch); // pitch is yaw ig
      //  BaseOpMode.addData("roll", roll);
        }
        if (!Double.isNaN(distance) && distance != 0) {
            actualDistance = alpha * (distance) + (1 - alpha) * actualDistance;
        }
        BaseOpMode.addData("tagDistance", actualDistance);

        if( team == BLUE && id == 20){
            heading = tx;
            distanceAway = Math.sqrt(Math.pow(0.46,2)+Math.pow(actualDistance,2)-(2*0.46*actualDistance*Math.cos(yaw)));
        } else if (team == RED && id == 24){
            heading = tx;
            distanceAway = actualDistance;
        }else{
            heading=0;
        }


        BaseOpMode.addData("distanceAway",distanceAway);
    }



    public double getBallSpeed(){
        //invert the regression comparing ball exit velocity to shooter RPM
        return ShooterDash.speedRegressionM * getShooterRPM();
    }

    public void setHoodAngleBasedOnTargetShotAngle(double angle){
        //This ignores the effect initial velocity has on angle. If we are having issues targeting at very low or high velocities, try to account for that
        //Values obtained from regression
        //hood.setPositionInterpolated((Math.asin((angle-52.7762)/18.1541)-163.65723)/4.04363);
        hood.setPositionInterpolated(angle);
    }

    public double getHoodAngle(){
        //add fancy math (High Case)
        //This is for the not-moving case
        xPosition = distanceAway;
        double ballSpeed = getBallSpeed();
        BaseOpMode.addData("ballSpeed", ballSpeed);
        double t1 = xPosition*Math.pow(ballSpeed,2);
        double t2 = Math.pow(t1,2);
        double t3 = Constants.g*Math.pow(xPosition,2)/2;
        double t4 = t3 + 0.75*Math.pow(ballSpeed,2);
        double t5 = Constants.g*Math.pow(xPosition,2);

        double input = (t1 - Math.sqrt(t2-(4*t3*t4)))/t5;
        double hoodAngle = Math.toDegrees(Math.atan(input));

        BaseOpMode.addData("input", input);
        BaseOpMode.addData("t1",t1);
        BaseOpMode.addData("t2",t2);
        BaseOpMode.addData("t3",t3);
        BaseOpMode.addData("t4",t4);
        BaseOpMode.addData("t5",t5);




        if(Double.isNaN(hoodAngle)){
            //this is the case where the equation returns NaN (it couldn't hit the target)
            //In this case, it declares that it can't shoot and returns the highest possible angle
            hoodCanShoot = false;
            return 60;
        }
        else{
            BaseOpMode.addData("targetAngle", hoodAngle);
            hoodCanShoot = true;
            return Range.clip(hoodAngle, 33, 65);
        }
    }

    public boolean canRobotShoot(){
        return (hoodCanShoot && turretCanShoot);
    }

    public ArrayList<BallColors> getCurrentRamp(){
        return currentRamp;
    }
    public boolean addBall(BallColors ballColor){
        if (currentRamp.size() > 9){
            currentRamp.add(ballColor);
            return true;
        }
        return false;
    }

    public void removeLastBall(){
        currentRamp.remove(currentRamp.size()-1);
    }

    public void clearRamp(){
        currentRamp.clear();
    }

    public BallColors[] getNextThree(){
        BallColors[] nextThree;
        nextThree = new BallColors[3];
        if (currentRamp.size() == 0 || currentRamp.size() == 3 || currentRamp.size() == 6){
            nextThree[0] = pattern[0];
            nextThree[1] = pattern[1];
            nextThree[2] = pattern[2];
        }
        else if (currentRamp.size() == 1 || currentRamp.size() == 4 || currentRamp.size() == 7){
            nextThree[0] = pattern[1];
            nextThree[1] = pattern[2];
            nextThree[2] = pattern[0];
        }
        else if (currentRamp.size() == 2 || currentRamp.size() == 5 || currentRamp.size() == 8){
            nextThree[0] = pattern[2];
            nextThree[1] = pattern[0];
            nextThree[2] = pattern[1];
        }

        return nextThree;
    }


    @Override
    public void updateSensors() {

    }

    @Override
    public void update() {
        work();
        updateTargeting();
    }

    public enum ShooterStates{
        ACTIVE, NOTACTIVE, OBELISK, SHOOTERTESTING;
    }
    public ShooterStates getState(){
        return shooterState;
    }

    public void setState(ShooterStates state){
        shooterState = state;
    }

    public enum BallColors{
        GREEN, PURPLE
    }

    public enum ShotType{
        HIGH, LOW
    }
}

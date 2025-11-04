// Primary Author: Dylan Cook
package org.firstinspires.ftc.teamcode.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Constants.Team.BLUE;
import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Constants.Team.RED;

import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Constants.biasterm;
import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Constants.goalAprilTagHeight;
import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Constants.limelightAngleOffset;
import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Constants.limelightLensHeightFromGround;
import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Constants.tyAlpha;
import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Constants.tyFiltered;


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
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Constants;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Hardware;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Control.PID;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Dash.PIDTuningDash;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Dash.ShooterDashClass;
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
    double tx;
    ShooterStates shooterState;
    TurretState turretState;
    double ticksPerRotation;
    double turretStartAngle;
    double turretTargetAngle;
    double turretError;
    Servo readyToShootIndicator;
    Servo angleWrapWarningLight;

    Limelight3A limelight;

    boolean hoodCanShoot = false;
    boolean turretCanShoot = false;
    double turretMarginForError = .15;

    //This has the middle of its range of motion as zero, and it can go this far in EITHER DIRECTION,
    //notated by setting it as negative or positive
    double turretRangeOfMotion = 3.5;

    double turretMaxRotation;
    ElapsedTime warningLightTimer;
    Constants.Team team;

    double degreesYtoApriltag;
    double radsYtoApriltag;
    double distanceAway;


    // Calibration state variables
    private int calibrationFrameCount = 0;
    private double runningOffsetSum = 0;
    private boolean offsetCalibrated = false;



    public Shooter(HardwareMap hardwareMap, double turretStartAngle, Constants.Team team){
        //Things are commented to prepare for the first tests of the shooter where we will only have the flywheel.

        shooter1 = new Motor(Hardware.shooter1,false, true);
        shooter2 = new Motor(Hardware.shooter2, false, true);
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
        turretState = TurretState.ACTIVE;
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
                updateTagDistanceHybridCorrected();
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
        shooter2.setPower(correction);
    }

    public int getPattern(){
        int id = 0;
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            id = fiducial.getFiducialId();
        }
        return id;

    }

    double turretResetTargetAngle = 0;
    double correction = 0;
    public void updateTurret(){
        double currentTurretAngle = turret.encoder.getPosition();
        //convert this to radians and wrap the angle
        currentTurretAngle = currentTurretAngle * (2*Math.PI/ticksPerRotation);

        //DONT angle wrap because the wiring means we can't actually spin around multiple times

        turretPDL.setConstants(0,0,0);
        switch (turretState){
            case ACTIVE:
                //if it is attempting to go outside its range of motion, switch to the resetting state
                if ((turretError + currentTurretAngle) > turretRangeOfMotion){
                    turretResetTargetAngle = turretRangeOfMotion-(2*Math.PI);
                    setTurretState(TurretState.RESETTING);
                } else if ((turretError + currentTurretAngle) < -turretRangeOfMotion)  {
                    turretResetTargetAngle = -turretRangeOfMotion+(2*Math.PI);
                    setTurretState(TurretState.RESETTING);
                } else {
                    correction = turretPDL.getCorrection(turretError);
                    turret.setPower(correction);

                    if (Math.abs(turretError) < turretMarginForError){
                        turretCanShoot = true;
                    } else {
                        turretCanShoot = false;
                    }
                }
                break;
            case RESETTING:
                if (false){

                } else {
                    correction = turretPDL.getCorrection(currentTurretAngle, turretResetTargetAngle);
                    
                }
        }
    }
    public void setTurretState(TurretState state){
        turretState = state;
    }

    double filteredRPM = 0;
    double rpmAlpha = 0.2;
    public double getShooterRPM(){
        double rpm = (Math.abs(shooter1.getVelocity()/ShooterTest.ShooterDash.ticksPerRotation*60)+Math.abs(shooter2.getVelocity()/ShooterTest.ShooterDash.ticksPerRotation*60))/2;

        filteredRPM = alpha*(rpm) + (1-rpmAlpha) * filteredRPM;
        if(shooter1.getVelocity() > 0){
        return filteredRPM;
        } else {return 0;}
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

        //This is the target angle relative to facing directly at the aprilTag
        //yaw is current angle of the aprilTag relative to the shooter
        //This uses the law of sines to find the target angle of the robot relative to the april tag
        //turretTargetAngle = Math.asin((0.46*Math.sin(yaw)/distanceAway));
        //turretError = turretTargetAngle - tx;




        //updateTurret();
        /*

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
            this.tx = tx;
            distanceAway = Math.sqrt(Math.pow(0.46,2)+Math.pow(actualDistance,2)-(2*0.46*actualDistance*Math.cos(yaw)));
        } else if (team == RED && id == 24){
            this.tx = tx;
            distanceAway = actualDistance;
        }else{
            this.tx =0;
        }


        BaseOpMode.addData("distanceAway",distanceAway);
    }

    double yaw;
    public void updateTagDistanceHybridCorrected() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            BaseOpMode.addData("HybridDistance", "No valid tag");
            return;
        }

        List<LLResultTypes.FiducialResult> fids = result.getFiducialResults();
        if (fids == null || fids.isEmpty()) return;

        for (LLResultTypes.FiducialResult fid : fids)
            if ((fid.getFiducialId() == 20 && team == BLUE) || (fid.getFiducialId() == 24 && team == RED)) {


                // Smooth ty
                double tx = result.getTx();
                tyFiltered = tyAlpha * tx + (1 - tyAlpha) * tyFiltered;

                // Dynamic angle compensation beyond ~2.8 m
                // Start with baseline Limelight mount angle
                double dynamicAngleOffset = limelightAngleOffset;

                // Gradually increase virtual mount angle up to +3.2° by 3.2 m
                // (simulates camera calibration bias at long range)
                if (actualDistance > 2.8) {
                    double ramp = Range.clip((actualDistance - 2.8) * 8.0, 0.0, 3.2); // 8°/m ramp
                    dynamicAngleOffset += ramp;
                }

                // Raw trig distance (m)
                double totalAngle = dynamicAngleOffset + tyFiltered;
                double trigDist = (goalAprilTagHeight - limelightLensHeightFromGround)
                        / Math.tan(Math.toRadians(totalAngle));

                // Mild linear correction beyond 2.3 m
                double correctedDist = trigDist;
                if (trigDist > 2.3) {
                    double bias = biasterm * (trigDist - 2.3);
                    correctedDist = trigDist - bias;
                }

                // pose-based cross-check beyond ~3.2 m
                double x = fid.getRobotPoseTargetSpace().getPosition().x;
                double z = fid.getRobotPoseTargetSpace().getPosition().z;
                double poseDist = Math.sqrt(x * x + z * z);
                boolean poseValid = poseDist > 0.3 && poseDist < 5.0;

                // Blend in up to 50% of pose data between 3.2–4.0 m
                double wPose = poseValid ? Range.clip((correctedDist - 3.2) / 0.8, 0.0, 0.5) : 0.0;
                double blended = (1 - wPose) * correctedDist + wPose * poseDist;

                yaw = 180 - Math.abs(fid.getCameraPoseTargetSpace().getOrientation().getPitch(AngleUnit.DEGREES) - 3);
                yaw = Math.toRadians(yaw);
                // Final smoothing filter for stability
                actualDistance = alpha * blended + (1 - alpha) * actualDistance;

                ///Heading and telemetry
                this.tx = result.getTx();
                distanceAway = Math.sqrt(Math.pow(0.46, 2) + Math.pow(actualDistance, 2) - 2 * 0.46 * actualDistance * Math.cos(yaw));
                this.tx = tx;

                BaseOpMode.addData("tyFiltered", tyFiltered);
                BaseOpMode.addData("DynamicAngleOffset", dynamicAngleOffset);
                BaseOpMode.addData("TrigDist(m)", trigDist);
                BaseOpMode.addData("CorrectedDist(m)", correctedDist);
                BaseOpMode.addData("PoseDist(m)", poseDist);
                BaseOpMode.addData("FinalDist(m)", actualDistance);
                BaseOpMode.addData("adjustedDistance", distanceAway);
            }
    }



    public double getBallSpeed(){
        //invert the regression comparing ball exit velocity to shooter RPM
        return ShooterDashClass.speedRegressionM * getShooterRPM();
    }
    public double getTargetBallSpeed(){
        //invert the regression comparing ball exit velocity to shooter RPM
        return ShooterDashClass.speedRegressionM * getTargetShooterRPM();
    }

    public void setHoodAngleBasedOnTargetShotAngle(double angle){
        //This ignores the effect initial velocity has on angle. If we are having issues targeting at very low or high velocities, try to account for that
        //Values obtained from regression
        //hood.setPositionInterpolated((Math.asin((angle-52.7762)/18.1541)-163.65723)/4.04363);
        hood.setPositionInterpolated(angle);
    }

    double hoodAngle;

    public double getHoodAngle(){
        //add fancy math (High Case)
        //This is for the not-moving case
        xPosition = distanceAway;
        double ballSpeed = getTargetBallSpeed();
        BaseOpMode.addData("ballSpeed", ballSpeed);
        double t1 = xPosition*Math.pow(ballSpeed,2);
        double t2 = Math.pow(t1,2);
        double t3 = Constants.g*Math.pow(xPosition,2)/2;
        double t4 = t3 + 0.75*Math.pow(ballSpeed,2);
        double t5 = Constants.g*Math.pow(xPosition,2);
        double input;
        if (distanceAway > 1){
        input = (t1 - Math.sqrt(t2-(4*t3*t4)))/t5;
        }
        else {input = t1 + Math.sqrt(t2-(4*t3*t4))/t5;}

        BaseOpMode.addData("input", input);
        BaseOpMode.addData("t1",t1);
        BaseOpMode.addData("t2",t2);
        BaseOpMode.addData("t3",t3);
        BaseOpMode.addData("t4",t4);
        BaseOpMode.addData("t5",t5);

        //If this works, if it can't find a new angle, it will return the last good angle.


        if(Double.isNaN(input)){
            //this is the case where the equation returns NaN (it couldn't hit the target)
            //In this case, it declares that it can't shoot and returns the last good angle
            hoodCanShoot = false;
            if (Double.isNaN(hoodAngle)){
                return 60;
            }
            else {
                return hoodAngle;
            }
        }
        else{
            BaseOpMode.addData("targetAngle", hoodAngle);
            hoodCanShoot = true;
            hoodAngle = Math.toDegrees(Math.atan(input));
            return Range.clip(hoodAngle, 33, 65);
        }
    }


    public void selectRPM(){
        //Populate this with all the RPM Ranges
        if (distanceAway < 1){
            targetShooterRPM = 1180;
        }
    }
    public boolean canRobotShoot(){
        return (hoodCanShoot && turretCanShoot);
    }



    @Override
    public void updateSensors() {

    }

    @Override
    public void update() {
        work();
        updateTagDistanceHybridCorrected();
        //updateTargeting();
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

    public enum TurretState{
        ACTIVE, RESETTING
    }
}

package org.firstinspires.ftc.teamcode.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Constants.Team.BLUE;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Constants.Team.RED;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Constants.team;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.teamcode.Utilities.Control.PID;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Dash.DashPositions;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Dash.PIDTuningDash;
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


    public Shooter(HardwareMap hardwareMap, double turretStartAngle, Constants.Team team){
        //Things are commented to prepare for the first tests of the shooter where we will only have the flywheel.

        shooter1 = new Motor(Hardware.shooter1);
        shooter2 = new Motor(Hardware.shooter2);
        //hood = new Servos.Hood();
        //turret = new Motor(Hardware.turret);
        //turretEncoder = hardwareMap.get(AnalogInput.class, "turretEncoder");
        shooterPDF = new PID(0,0,0);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // make number higher to get more data
        limelight.pipelineSwitch(1);
        limelight.start();

        limelight.reloadPipeline();
        this.team = team;

        pattern = new BallColors[3];

        currentRamp = new ArrayList<BallColors>();
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
                updateShooter();
                //hood.setPositionInterpolated(DashPositions.servoTest);
                targetShooterRPM = DashPositions.dashShooterRPM;

        }
    };



    public void updateShooter(){
        shooterPDF.setConstants(PIDTuningDash.ShooterP,0,PIDTuningDash.ShooterD);
        shooterPDF.setFeedForward(PIDTuningDash.ShooterF);

        //if I understand correctly, gobilda's documentation says that a bare motor has 28 ticks per revolution, and we're running with a 1 to 1 gear ratio.
        double shooterRPM = ((shooter1.getVelocity()/28)+(shooter2.getVelocity()/28))/2;
        double correction = shooterPDF.getCorrection(shooterRPM,targetShooterRPM);

        shooter1.setPower(correction);
        shooter2.setPower(correction);
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

    public double setTargetBallSpeed(double speed){
        //make a regression comparing ball exit velocity to shooter RPM
        targetShooterRPM = speed;
        return targetShooterRPM;
    }

    public void aim(){

        hood.setPositionInterpolated(getHoodAngleHigh());
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int id = fiducial.getFiducialId(); // The ID number of the fiducial
            double degreesXtoApriltag = fiducial.getTargetXDegrees(); //gets angle to limelight along x plane

            double ty = limelight.getLatestResult().getTy(); // gets degrees to crosshair from primary target along y axis
            double tx = limelight.getLatestResult().getTx();// gets degrees to crosshair from primary target along x axis

            if( team == BLUE || id == 20){
                heading = tx;
            } else if (team == RED || id == 24){
                heading = tx;
            }
        }


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
    }



    public double getBallSpeed(){
        //invert the regression comparing ball exit velocity to shooter RPM
        return 1;
    }

    public double getHoodAngleHigh(){
        //add fancy math (High Case)
        double hoodAngle = 1;

        if(Double.isNaN(hoodAngle) || (hoodAngle < 31 || hoodAngle > 60)){
            //this is the case where the equation returns NaN (it couldn't hit the target)
            //In this case, it declares that it can't shoot and returns the highest possible angle
            hoodCanShoot = false;
            return 60;
        }
        else {
            //in this case it can shoot, and it returns the angle to shoot at.
            hoodCanShoot = true;
            return hoodAngle;
        }
    }
    public double getHoodAngleLow(){
        //add fancy math (Low Case)
        double hoodAngle = 1;

        if(Double.isNaN(hoodAngle) || (hoodAngle < 31 || hoodAngle > 60)){
            //this is the case where the equation returns NaN (it couldn't hit the target)
            //In this case, it declares that it can't shoot and returns the highest possible angle
            hoodCanShoot = false;
            return 31;
        }
        else {
            //in this case it can shoot, and it returns the angle to shoot at.
            hoodCanShoot = true;
            return hoodAngle;
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

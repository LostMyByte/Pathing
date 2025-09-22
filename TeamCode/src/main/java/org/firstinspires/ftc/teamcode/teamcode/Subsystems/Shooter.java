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


    public Shooter(HardwareMap hardwareMap, double turretStartAngle){
        //Things are commented to prepare for the first tests of the shooter where we will only have the flywheel.

        shooter1 = new Motor(Hardware.shooter1);
        shooter2 = new Motor(Hardware.shooter2);
        //hood = new Servos.Hood();
        //turret = new Motor(Hardware.turret);
        //turretEncoder = hardwareMap.get(AnalogInput.class, "turretEncoder");
        shooterPDF = new PID(0,0,0);
        //turretPDL = new PID(0,0,0);
        //limelight = hardwareMap.get(Limelight3A.class, "limelight");
        //limelight.setPollRateHz(100); // make number higher to get more data
        //limelight.pipelineSwitch(0);
        //limelight.start();

        //limelight.reloadPipeline();

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
        hood.setPositionInterpolated(getHoodAngle());
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

    public double getHoodAngle(){
        //add fancy math
        double hoodAngle = 1;

        if(Double.isNaN(hoodAngle)){
            //this is the case where the equation returns NaN (it couldn't hit the target no matter what angle)
            //In this case, it declares that it can't shoot and returns the lowest possible angle
            hoodCanShoot = false;
            return 31;
        }

        if (hoodAngle < 31 || hoodAngle > 60){
            //this is the case where it could shoot, but the hood doesn't have enough range of motion
            //in this case, it declares that it can't shoot and returns the clipped angle
            hoodCanShoot = false;
        }
        else {
            //in this case it can shoot, and it returns the angle to shoot at.
            hoodCanShoot = true;
        }
        return Range.clip(hoodAngle, 31, 60);
    }

    public boolean canRobotShoot(){
        if (hoodCanShoot && turretCanShoot){
            return true;
        } else {
            return false;
        }
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
}

package org.firstinspires.ftc.teamcode.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Constants.Team.BLUE;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Constants.Team.RED;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Constants.team;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.teamcode.Utilities.Control.PID;
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
    ShooterStates shooterStates;
    double ticksPerRotation;
    double turretStartAngle;
    double turretTargetAngle;

    Limelight3A limelight;



    public Shooter(HardwareMap hardwareMap, double turretStartAngle){
        shooter1 = new Motor(Hardware.shooter1);
        shooter2 = new Motor(Hardware.shooter2);
        hood = new Servos.Hood();
        turret = new Motor(Hardware.turret);
        turretEncoder = hardwareMap.get(AnalogInput.class, "turretEncoder");
        shooterPDF = new PID(0,0,0);
        turretPDL = new PID(0,0,0);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // make number higher to get more data
        limelight.pipelineSwitch(0);
        limelight.start();

        limelight.reloadPipeline();

        this.turretStartAngle = turretStartAngle;
    }

    public void work(){
        switch (shooterStates){
            case ACTIVE:
                aim();
                updateShooter();
            case NOTACTIVE:
                aim();
            case OBELISK:

        }
    };



    public void updateShooter(){
        shooterPDF.setConstants(PIDTuningDash.ShooterP,0,PIDTuningDash.ShooterD);
        shooterPDF.setFeedForward(PIDTuningDash.ShooterF);

        //if I understand correctly, gobilda's documentation says that a bare motor has 28 ticks per revolution
        double shooterRPM = ((shooter1.getVelocity()/28)+(shooter2.getVelocity()/28))/2;
        double correction = shooterPDF.getCorrection(shooterRPM,targetShooterRPM);

        shooter1.setPower(correction);
        shooter2.setPower(correction);
    }

    public void updateTurret(){
        double currentAngle = turret.encoder.getPosition();
        //convert this to radians and wrap the angle
        currentAngle = currentAngle * (2*Math.PI/ticksPerRotation);

        while (currentAngle > Math.PI){
            currentAngle -= 2*Math.PI;
        }
        while (currentAngle < -Math.PI){
            currentAngle += 2*Math.PI;
        }

        turretPDL.setConstants(0,0,0);
        turretPDL.getCorrectionHeading(currentAngle,turretTargetAngle);



        turret.setPower(turretPDL.getCorrectionHeading(currentAngle,turretTargetAngle));
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

        while (turretTargetAngle > Math.PI){
            turretTargetAngle -= 2*Math.PI;
        }
        while (turretTargetAngle < -Math.PI){
            turretTargetAngle += 2*Math.PI;
        }
        updateTurret();
    }



    public double getBallSpeed(){
        //invert the regression comparing ball exit velocity to shooter RPM
        return 1;
    }

    public double getHoodAngle(){
        return 1;
    }

    @Override
    public void updateSensors() {

    }

    @Override
    public void update() {
        work();
    }

    public enum ShooterStates{
        ACTIVE, NOTACTIVE, OBELISK;
    }
}

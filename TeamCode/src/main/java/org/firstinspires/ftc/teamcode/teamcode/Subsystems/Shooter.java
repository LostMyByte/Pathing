package org.firstinspires.ftc.teamcode.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.teamcode.Utilities.Control.PID;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Dash.PIDTuningDash;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.HardwareDevices.Motor;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.HardwareDevices.Servo;

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



    public Shooter(HardwareMap hardwareMap, double turretStartAngle){
        shooter1 = new Motor(Hardware.shooter1);
        shooter2 = new Motor(Hardware.shooter2);
        hood = new Servos.Hood();
        turret = new Motor(Hardware.turret);
        turretEncoder = hardwareMap.get(AnalogInput.class, "turretEncoder");
        shooterPDF = new PID(0,0,0);
        turretPDL = new PID(0,0,0);

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
        double angle = turret.encoder.getPosition();
        //convert this to radians and wrap the angle
        angle = angle * (2*Math.PI/ticksPerRotation);

        while (angle > Math.PI){
            angle -= 2*Math.PI;
        }
        while (angle < -Math.PI){
            angle += 2*Math.PI;
        }

        turretPDL.setConstants(0,0,0);
        turretPDL.getCorrectionHeading(angle,turretTargetAngle);

        turret.setPower(turretPDL.getCorrectionHeading(angle,turretTargetAngle));
    }

    public double setTargetBallSpeed(double speed){
        //make a regression comparing ball exit velocity to shooter RPM
        targetShooterRPM = speed;
        return targetShooterRPM;
    }

    public void aim(){
        hood.setPositionInterpolated(getHoodAngle());


        turretTargetAngle = Math.asin(xVelocity/getBallSpeed()) - heading;

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

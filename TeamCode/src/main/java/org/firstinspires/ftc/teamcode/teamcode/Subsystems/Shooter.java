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
    double targetShooterRPM;
    Drivetrain drivetrain;


    public Shooter(HardwareMap hardwareMap){
        shooter1 = new Motor(Hardware.shooter1);
        shooter2 = new Motor(Hardware.shooter2);
        hood = new Servos.Hood();
        turret = new Motor(Hardware.turret);
        turretEncoder = hardwareMap.get(AnalogInput.class, "turretEncoder");
        shooterPDF = new PID(0,0,0);

        drivetrain = new Drivetrain(hardwareMap,0);

    }



    public void updateShooter(){
        shooterPDF.setConstants(PIDTuningDash.ShooterP,0,PIDTuningDash.ShooterD);
        shooterPDF.setFeedForward(PIDTuningDash.ShooterF);

        //if I understand correctly, gobilda's documentation says that a bare motor has 28 ticks per revolution
        double shooterRPM = ((shooter1.getVelocity()/28)+(shooter2.getVelocity()/28))/2;
        double correction = shooterPDF.getCorrection(shooterRPM,targetShooterRPM);

        shooter1.setPower(correction);
        shooter2.setPower(correction);
    }

    public double setTargetBallSpeed(double speed){
        //make a regression comparing ball exit velocity to shooter RPM
        targetShooterRPM = speed;
        return targetShooterRPM;
    }



    @Override
    public void updateSensors() {

    }

    @Override
    public void update() {
        updateShooter();
    }

}

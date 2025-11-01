package org.firstinspires.ftc.teamcode.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.TeliOp.ColorRangefinder;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Hardware;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.HardwareDevices.Motor;

public class IntakeMagazine extends Subsystem{

    CRServo mecanum1;
    CRServo mecanum2;
    Servos.MagazineL magazineL;
    Servos.MagazineR magazine;
    Servos.ShooterDoor shooterDoor;
    Motor intakeMotor;


    public IntakeMagazine(HardwareMap hardwareMap){
        mecanum1 = hardwareMap.get(CRServo.class, Hardware.mecanum1);
        mecanum2 = hardwareMap.get(CRServo.class, Hardware.mecanum2);
        magazineL = new Servos.MagazineL();
        magazine = new Servos.MagazineR();
        shooterDoor = new Servos.ShooterDoor();
        intakeMotor = new Motor(Hardware.intake);
        magazine.pair(magazineL);
    }

    public void startIntake(){
        intakeMotor.setPower(0.7);
        mecanum1.setPower(0.5);
        mecanum2.setPower(0.5);
        magazine.secondBall();
    }
    public void stopIntake(){
        intakeMotor.setPower(0);
        magazine.firstBall();
    }



    public void shoot(){
        shooterDoor.open();
    }

    public void update(){}
    public void updateSensors(){}
    public enum IntakeMagazineStates{
        INTAKEACTIVE, INTAKENOTACTIVE, LOADED1, LOADED2, LOADED3, SHOOTING1, SHOOTING2, SHOOTING3
    }


}

package org.firstinspires.ftc.teamcode.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Hardware;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.HardwareDevices.Motor;

import java.lang.reflect.Array;
import java.util.ArrayList;

public class IntakeMagazine extends Subsystem{

    CRServo mecanum1;
    CRServo mecanum2;
    Servos.MagazineL magazineL;
    Servos.MagazineR magazine;
    Servos.ShooterDoor shooterDoor;
    Motor intakeMotor;
    IntakeMagazineStates state = IntakeMagazineStates.INIT;
    ElapsedTime timer;
    BallColors[] indexer;


    public IntakeMagazine(HardwareMap hardwareMap){
        mecanum1 = hardwareMap.get(CRServo.class, Hardware.mecanum1);
        mecanum2 = hardwareMap.get(CRServo.class, Hardware.mecanum2);
        magazineL = new Servos.MagazineL();
        magazine = new Servos.MagazineR();
        shooterDoor = new Servos.ShooterDoor();
        intakeMotor = new Motor(Hardware.intake);
        magazine.pair(magazineL);
        timer = new ElapsedTime();
    }

    public void work(){
        switch(state){
            case INIT:
                break;
            case LOADED1:
                loaded1();
                break;
            case LOADED2:
                loaded2();
                break;
            case LOADED3:
                loaded3();
                break;
            case SHOOTING1:
                shooting1();
                break;
            case SHOOTING2:
                shooting2();
                break;
            case SHOOTING3:
                shooting3();
                break;
            case INTAKEACTIVE:
                intaking();
                break;
        }
    }

    public void loaded1(){
        magazine.firstBall();
        intakeMotor.setPower(0);
        setMecanumPower(0.2);
        shooterDoor.closed();
    }
    public void loaded2(){
        magazine.secondBall();
        intakeMotor.setPower(0);
        setMecanumPower(0.2);
        shooterDoor.closed();
    }
    public void loaded3(){
        magazine.thirdBall();
        intakeMotor.setPower(0);
        setMecanumPower(0.2);
        shooterDoor.closed();
    }
    public void shooting1(){
        magazine.firstBall();
        intakeMotor.setPower(0);
        setMecanumPower(1);
        shooterDoor.open();
        if (timer.seconds() > 1){
            indexer[0] = indexer[1];
            indexer[1] = indexer[2];
            indexer[2] = BallColors.NONE;
            setState(IntakeMagazineStates.LOADED1);
        }
    }
    public void shooting2(){
        magazine.secondBall();
        intakeMotor.setPower(0);
        setMecanumPower(1);
        shooterDoor.open();
        if (timer.seconds() > 1){
            indexer[1] = indexer[2];
            indexer[2] = BallColors.NONE;
            setState(IntakeMagazineStates.LOADED2);
        }
    }
    public void shooting3(){
        magazine.thirdBall();
        intakeMotor.setPower(0);
        setMecanumPower(1);
        shooterDoor.open();
        if (timer.seconds() > 1){
            indexer[2] = BallColors.NONE;
            setState(IntakeMagazineStates.LOADED3);
        }
    }

    public void intaking(){
        intakeMotor.setPower(0.7);
        setMecanumPower(0.5);
        magazine.secondBall();
        shooterDoor.closed();
    }

    public void setMecanumPower(double power){
        mecanum1.setPower(power);
        mecanum2.setPower(power);
    }



    public void update(){}
    public void updateSensors(){}
    public enum IntakeMagazineStates{
        INTAKEACTIVE, LOADED1, LOADED2, LOADED3, SHOOTING1, SHOOTING2, SHOOTING3, INIT,
    }

    public enum BallColors{
        GREEN, PURPLE, UNKNOWN, NONE
    }

    public void setState(IntakeMagazineStates state){
        this.state = state;
        timer.reset();
    }




}

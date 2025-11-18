package org.firstinspires.ftc.teamcode.teamcode.Subsystems;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Hardware;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.HardwareDevices.Motor;

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
    NormalizedColorSensor[] colorSensors = new NormalizedColorSensor[2];
    final float[] hsv = new float[3];
    SensorReadStates sensorReadState = SensorReadStates.NOTREADING;
    int readNum = 0;

    public IntakeMagazine(HardwareMap hardwareMap){
        mecanum1 = hardwareMap.get(CRServo.class, Hardware.mecanum1);
        mecanum2 = hardwareMap.get(CRServo.class, Hardware.mecanum2);
        magazineL = new Servos.MagazineL();
        magazine = new Servos.MagazineR();
        shooterDoor = new Servos.ShooterDoor();
        intakeMotor = new Motor(Hardware.intake);
        magazine.pair(magazineL);
        timer = new ElapsedTime();
        colorSensors[0] = hardwareMap.get(NormalizedColorSensor.class, "colorSensor0");
        colorSensors[1] = hardwareMap.get(NormalizedColorSensor.class, "colorSensor1");
        colorSensors[2] = hardwareMap.get(NormalizedColorSensor.class, "colorSensor2");
    }

    public void work(){
        switch(state){
            case INIT:
                break;
            case LOADED0:
                loaded0();
                break;
            case LOADED1:
                loaded1();
                break;
            case LOADED2:
                loaded2();
                break;
            case SHOOTING0:
                shooting0();
                break;
            case SHOOTING1:
                shooting1();
                break;
            case SHOOTING2:
                shooting2();
                break;
            case INTAKEACTIVE:
                intaking();
                break;
            case INTAKEREVERSED:
                intakeReversed();
                break;
        }
    }

    public void sensorStateMachine(){
        switch (sensorReadState){
            case READING:
                readSensors();
                break;
            case NOTREADING:
                break;
        }
    }

    public void readSensors(){
        for(int sensorNum = 0; sensorNum < colorSensors.length; sensorNum++){
            NormalizedRGBA colors = colorSensors[sensorNum].getNormalizedColors();
            Color.colorToHSV(colors.toColor(),hsv);
            if (hsv[0] > 200){
                indexer[sensorNum] = BallColors.PURPLE;
            } else if (hsv[0] > 100){
                indexer[sensorNum] = BallColors.GREEN;
            }
        }
        readNum++;
        if (readNum > 5){
            setSensorReadState(SensorReadStates.NOTREADING);
        }
    }


    public void loaded0(){
        magazine.ball0();
        intakeMotor.setPower(0);
        setMecanumPower(0.2);
        shooterDoor.closed();
    }
    public void loaded1(){
        magazine.ball1();
        intakeMotor.setPower(0);
        setMecanumPower(0.2);
        shooterDoor.closed();
    }
    public void loaded2(){
        magazine.ball2();
        intakeMotor.setPower(0);
        setMecanumPower(0.2);
        shooterDoor.closed();
    }
    public void shooting0(){
        magazine.ball0();
        intakeMotor.setPower(0);
        setMecanumPower(1);
        shooterDoor.open();
        if (timer.seconds() > 1){
            indexer[0] = indexer[1];
            indexer[1] = indexer[2];
            indexer[2] = BallColors.NONE;
            setState(IntakeMagazineStates.LOADED0);
        }
    }
    public void shooting1(){
        magazine.ball1();
        intakeMotor.setPower(0);
        setMecanumPower(1);
        shooterDoor.open();
        if (timer.seconds() > 1){
            indexer[1] = indexer[2];
            indexer[2] = BallColors.NONE;
            setState(IntakeMagazineStates.LOADED1);
        }
    }
    public void shooting2(){
        magazine.ball2();
        intakeMotor.setPower(0);
        setMecanumPower(1);
        shooterDoor.open();
        if (timer.seconds() > 1){
            indexer[2] = BallColors.NONE;
            setState(IntakeMagazineStates.LOADED2);
        }
    }

    public void intaking(){
        intakeMotor.setPower(0.7);
        setMecanumPower(0.5);
        magazine.ball1();
        shooterDoor.closed();
    }
    public void intakeReversed(){
        intakeMotor.setPower(-0.7);
        setMecanumPower(-0.5);
        magazine.ball1();
        shooterDoor.closed();
    }

    public void setMecanumPower(double power){
        mecanum1.setPower(power);
        mecanum2.setPower(power);
    }

    public boolean shootPurple(){
        if (indexer[0] == BallColors.PURPLE){
            setState(IntakeMagazineStates.SHOOTING0);
        } else if (indexer[1] == BallColors.PURPLE){
            setState(IntakeMagazineStates.SHOOTING1);
        } else if (indexer[2] == BallColors.PURPLE){
            setState(IntakeMagazineStates.SHOOTING2);
        } else {return false;}

        return true;
    }

    public boolean shootGreen(){
        if (indexer[0] == BallColors.GREEN){
            setState(IntakeMagazineStates.SHOOTING0);
        } else if (indexer[1] == BallColors.GREEN){
            setState(IntakeMagazineStates.SHOOTING1);
        } else if (indexer[2] == BallColors.GREEN){
            setState(IntakeMagazineStates.SHOOTING2);
        } else {return false;}

        return true;
    }





    public void update(){
        sensorStateMachine();
        work();
    }
    public void updateSensors(){}

    public enum IntakeMagazineStates{
        INTAKEACTIVE, INTAKEREVERSED,LOADED1, LOADED2, LOADED0, SHOOTING1, SHOOTING2, SHOOTING0, INIT, READINGSENSORS
    }

    public enum SensorReadStates{
        READING, NOTREADING
    }

    public enum BallColors{
        GREEN, PURPLE, NONE
    }

    public void setState(IntakeMagazineStates state){
        this.state = state;
        timer.reset();
    }

    public IntakeMagazineStates getState(){
        return state;
    }

    public void setSensorReadState(SensorReadStates sensorReadState) {
        this.sensorReadState = sensorReadState;
    }
}

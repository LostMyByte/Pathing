package org.firstinspires.ftc.teamcode.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.IntakeMagazine.IntakeMagazineStates.IDLE;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.IntakeMagazine.IntakeMagazineStates.INTAKEFRONT;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.IntakeMagazine.IntakeMagazineStates.INTAKEREAR;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.IntakeMagazine.IntakeMagazineStates.SHOOTING;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Hardware;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.HardwareDevices.Motor;

public class IntakeMagazine extends Subsystem{


    Motor frontIntake;
    Motor rearIntake;
    CRServo indexWheel;
    //the ramp closest to the back side of the robot
    Servos.FliPrampBack backRamp;
    //The ramp closest to the front side of the robot
    Servos.FliPrampFront frontRamp;

    IntakeMagazineStates state = IDLE;
    IntakeMagazineStates lastState = IDLE;

    IndexStates indexState = IndexStates.SCORE;

    ElapsedTime timer;
    
    NormalizedColorSensor[] colorSensors = new NormalizedColorSensor[3];
    TouchSensor[] breakBeams = new TouchSensor[3];
    boolean[] breakBeamReads = new boolean[3];
    boolean[] preShotBreakBeams = new boolean[3];
    final float[] hsv = new float[3];
    public boolean indexMode = false;
    ElapsedTime indexingTimer;
    boolean firstLoop1 = true;
    boolean firstLoop2 = true;

    public IntakeMagazine(HardwareMap hardwareMap){
        frontRamp = new Servos.FliPrampFront();
        backRamp = new Servos.FliPrampBack();
        frontIntake = new Motor(Hardware.frontIntake, true);
        rearIntake = new Motor(Hardware.rearIntake, false);
        indexWheel = hardwareMap.get(CRServo.class, Hardware.indexServo2);
        timer = new ElapsedTime();
        indexingTimer = new ElapsedTime();
        colorSensors[0] = hardwareMap.get(NormalizedColorSensor.class, "colorSensor0");
        colorSensors[1] = hardwareMap.get(NormalizedColorSensor.class, "colorSensor1");
        colorSensors[2] = hardwareMap.get(NormalizedColorSensor.class, "colorSensor2");
        breakBeams[0] = hardwareMap.get(TouchSensor.class, "breakbeam2");
        breakBeams[1] = hardwareMap.get(TouchSensor.class, "breakbeam0");
        breakBeams[2] = hardwareMap.get(TouchSensor.class, "breakbeam1");
    }


    //We start in the idle state. From there we can shoot or intake from either direction.

    public void work(){
        switch(state){
            case SHOOTING:
                shoot();
                break;
            case IDLE:
                idle();
                break;
            case INTAKEFRONT:
                intakeFront();
                break;
            case INTAKEREAR:
                intakeRear();
                break;
            case CLEAR:
                clearMagazine();
                break;
            case DONOTHING:
                break;
        }
    }

    public boolean[] getBreakBeamReads(){
        return breakBeamReads;
    }



    public void setIndexMode(boolean indexMode){
        //if we just switched from indexmode to not indexmode, so if the new indexmode is false and the old one is true
        if(!indexMode && this.indexMode){
            setIndexState(IndexStates.SCORE);
        }
        this.indexMode = indexMode;
        //this might cause issues so check this for edge cases.
        timer.reset();
    }

    public void handleRamps(RobotSide shootingSide){
        if(shootingSide == RobotSide.BACK){
            backRamp.flat();
            if (breakBeamReads[2]){
                frontRamp.shoot();
            } else {
                frontRamp.mid();
            }
        } else {
            frontRamp.flat();
            if (breakBeamReads[2]){
                backRamp.shoot();
            } else {
                backRamp.mid();
            }
        }
    }
    //release the shooter door and run both intakes to shoot. Then declare the loaded position empty, and
    //if we are not in index mode, load a new ball. If we are, switch to the idle state so avery can decide
    //what ball to shoot next

    public void shoot(){
        if (firstLoop1){
            preShotBreakBeams = breakBeamReads;
            firstLoop1 = false;
        }
        frontIntake.setPower(1);
        rearIntake.setPower(1);
        indexWheel.setPower(-1);
        //if all positions are full when we start shooting
        if (preShotBreakBeams[1] && preShotBreakBeams[0]){
            if (timer.seconds() > 0.7 && !breakBeamReads[2]){
                setState(IDLE);
            }
            else if (timer.seconds() < 0.5){
                handleRamps(RobotSide.BACK);
            } else {
                handleRamps(RobotSide.FRONT);
            }
        } else if (preShotBreakBeams[1]){
            handleRamps(RobotSide.BACK);
            if (timer.seconds() > 0.5){
                setState(IDLE);
            }
        }
        else if (preShotBreakBeams[0]) {
            handleRamps(RobotSide.FRONT);
            if (timer.seconds() > 0.5) {
                setState(IDLE);
            }
        } else {
            setState(IDLE);
        }
    }

    //This state assumes that we are not in index mode. Shut the shooter door and check if there is a ball
    //in either position. If there is, load it and switch to the idle state.


    //Do nothing, be ready to shoot.
    //if we are in index mode and there is a ball in the middle position, move it to the indexed position.
    //When it is in the indexed position, indicate that it is there, and declare the shooting position empty
    public void idle(){
        indexWheel.setPower(-.5);
        frontRamp.lessMid();
        backRamp.lessMid();
        //if there is a ball in the front intake
        if (!breakBeamReads[0] && firstLoop1){
            frontIntake.setPower(-.5);
        } else {
            frontIntake.setPower(0.2);
            firstLoop1 = false;
        }
        if (!breakBeamReads[1] && firstLoop2){
            rearIntake.setPower(-.5);
        } else {
            rearIntake.setPower(0.2);
            firstLoop2 = false;
        }

    }

    //if avery requests a green to be shot, check all the slots for a green, and if there is a green,
    //load it into the shooting position and switch to the shoot state.
    //If no greens are found, switch to idle mode
    //this state assumes we are in index mode
    //the next state does the same thing but for purple



    //run both intakes the same direction (one in, one out), in order to pass a ball from the currently
    //active intake into the opposite side intake. When there is a ball in the opposite side intake, stop
    //that intake.
    public void intakeFront(){
        indexWheel.setPower(-.5);
        if (getNumBalls() != 3) {
            frontIntake.setPower(.85);
            frontRamp.flat();
            backRamp.flat();
            //if there is a ball in the front intake, stop reversing that intake
            if (!breakBeamReads[1]) {
                rearIntake.setPower(0.1);
            } else {
                rearIntake.setPower(-0.5);
            }
        } else {
            rearIntake.setPower(0);
            frontIntake.setPower(0);
        }
    }
    public void intakeRear(){
        indexWheel.setPower(-.5);
        if (getNumBalls() != 3) {
            rearIntake.setPower(.85);
            frontRamp.flat();
            backRamp.flat();
            //if there is a ball in the front intake, stop reversing that intake
            if (!breakBeamReads[0]) {
                frontIntake.setPower(0.1);
            } else {
                frontIntake.setPower(-0.5);
            }
        } else {
            rearIntake.setPower(0);
            frontIntake.setPower(0);
        }

    }

    public void clearMagazine(){
        frontRamp.flat();
        backRamp.flat();
        rearIntake.setPower(-1);
        frontIntake.setPower(-1);
        if (timer.seconds() > 0.2){
            if (firstLoop1){
                setIndexState(IndexStates.INDEX);
                firstLoop1 = false;
            }
        }
        resetBreakBeams();
    }

    public void resetBreakBeams(){
        breakBeamReads[0] = false;
        breakBeamReads[1] = false;
        breakBeamReads[2] = false;
    }

    public int getNumBalls(){
        int num = 0;
        if (!breakBeamReads[0]){
            num++;
        } if (!breakBeamReads[1]){
            num++;
        } if (!breakBeamReads[2]){
            num++;
        }
        return num;
    }

    public void setIndexState(IndexStates state){
        indexState = state;
        indexingTimer.reset();
    }
    public void indexerStateMachine(){
        switch(indexState){
            case SCORE:
                if (indexingTimer.seconds() < 1){
                    indexWheel.setPower(-1);
                } else {
                    indexWheel.setPower(0);
                }
                break;
            case INDEX:
                if (indexingTimer.seconds() < 1){
                    indexWheel.setPower(1);
                } else {
                    indexWheel.setPower(0);
                }
        }
    }


    @Override
    public void update() {
        work();
        BaseOpMode.addData("beam0", breakBeamReads[0]);
        BaseOpMode.addData("beam1", breakBeamReads[1]);
        BaseOpMode.addData("beam2", breakBeamReads[2]);
        BaseOpMode.addData("numBalls", getNumBalls());
    }

    @Override
    public void updateSensors(){
        for(int sensorNum = 0; sensorNum < breakBeams.length && state != SHOOTING; sensorNum++){
            breakBeamReads[sensorNum] = breakBeams[sensorNum].isPressed();
        }
    }

    /*@Override
    public void update() {
        work();
    }*/

    public enum IntakeMagazineStates{
        IDLE, INTAKEFRONT, INTAKEREAR, SHOOTING, CLEAR, LOADANDSHOOTUNINDEXED, DONOTHING
    }

    public enum BallColors{
        GREEN, PURPLE, NONE
    }

    public enum RobotSide{FRONT, BACK}
    public enum IndexStates{
        INDEX,SCORE
    }

    public void setState(IntakeMagazineStates state){
        lastState = this.state;
        this.state = state;
        timer.reset();
        firstLoop1 = true;
        firstLoop2 = true;
    }

    public IntakeMagazineStates getState(){
        return state;
    }
}

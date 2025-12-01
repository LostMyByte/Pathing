package org.firstinspires.ftc.teamcode.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.IntakeMagazine.IntakeMagazineStates.IDLE;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.IntakeMagazine.IntakeMagazineStates.INTAKEFRONT;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.IntakeMagazine.IntakeMagazineStates.LOAD;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.IntakeMagazine.IntakeMagazineStates.SHOOTING;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Hardware;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.HardwareDevices.Motor;

public class IntakeMagazine extends Subsystem{


    Servos.ShooterDoor shooterDoor;
    Motor frontIntake;
    Motor rearIntake;
    Servos.indexIn indexServo;
    IntakeMagazineStates state = IDLE;
    ElapsedTime timer;

    //slot 0 is front intake, slot 1 is rear intake, slot 2 is the index slot, and slot 3 is the loaded slot
    BallColors[] indexer = new BallColors[3];
    NormalizedColorSensor[] colorSensors = new NormalizedColorSensor[2];
    TouchSensor[] breakBeams = new TouchSensor[2];
    boolean[] breakBeamReads = new boolean[3];
    final float[] hsv = new float[3];
    public boolean indexMode = false;

    public IntakeMagazine(HardwareMap hardwareMap){
        shooterDoor = new Servos.ShooterDoor();
        frontIntake = new Motor(Hardware.frontIntake);
        rearIntake = new Motor(Hardware.rearIntake);
        timer = new ElapsedTime();
        colorSensors[0] = hardwareMap.get(NormalizedColorSensor.class, "colorSensor0");
        colorSensors[1] = hardwareMap.get(NormalizedColorSensor.class, "colorSensor1");
        colorSensors[2] = hardwareMap.get(NormalizedColorSensor.class, "colorSensor2");
        breakBeams[0] = hardwareMap.get(TouchSensor.class, "breakbeam0");
        breakBeams[1] = hardwareMap.get(TouchSensor.class, "breakbeam1");
        breakBeams[2] = hardwareMap.get(TouchSensor.class, "breakbeam2");
    }


    //We start in the idle state. From there we can shoot or intake from either direction.

    public void work(){
        switch(state){
            case SHOOTING:
                shoot();
                break;
            case LOAD:
                load();
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
            case LOADGREEN:
                loadGreen();
                break;
            case LOADPURPLE:
                loadPurple();
                break;
            case CLEAR:
                clearMagazine();
                break;
            case LOADANDSHOOTUNINDEXED:
                loadAndShootUnindexed();
                break;
        }
    }

    public void setIndexMode(boolean indexMode){
        this.indexMode = indexMode;
    }

    //release the shooter door and run both intakes to shoot. Then declare the loaded position empty, and
    //if we are not in index mode, load a new ball. If we are, switch to the idle state so avery can decide
    //what ball to shoot next

    public void shoot(){
        rearIntake.setPower(1);
        frontIntake.setPower(1);
        shooterDoor.open();
        if (timer.seconds() > 0.5){
            breakBeamReads[3] = false;
            indexer[3] = BallColors.NONE;
            if (!indexMode) {
                setState(LOAD);
            }
            else {
                setState(IDLE);
            }
        }
    }

    //This state assumes that we are not in index mode. Shut the shooter door and check if there is a ball
    //in either position. If there is, load it and switch to the idle state.

    public void load(){
        shooterDoor.closed();
        if(breakBeamReads[0]){
            frontIntake.setPower(1);
            rearIntake.setPower(0);
            if (timer.seconds() > 0.5){
                breakBeamReads[3] = true;
                breakBeamReads[0] = false;
                setState(IDLE);
            }
        } else if (breakBeamReads[1]){
            rearIntake.setPower(1);
            frontIntake.setPower(0);
            if (timer.seconds() > 0.5){
                breakBeamReads[3] = true;
                breakBeamReads[1] = false;
                setState(IDLE);
            }
        }
    }

    //Do nothing, be ready to shoot.
    //if we are in index mode, move the indexer to the indexed position to check if there is a ball there.
    //if there is, keep the indexer there. If there is no ball, move the indexer back into the home position.
    public void idle(){
        frontIntake.setPower(0);
        rearIntake.setPower(0);
        shooterDoor.closed();
        if (indexMode){
            if (breakBeamReads[2]){
                indexServo.index();
            }
        }
    }

    //if avery requests a green to be shot, check all the slots for a green, and if there is a green,
    //load it into the shooting position and switch to the shoot state.
    //If no greens are found, switch to idle mode
    //this state assumes we are in index mode
    //the next state does the same thing but for purple
    public void loadGreen(){
        shooterDoor.closed();
        if (indexer[3] == BallColors.GREEN){
            setState(SHOOTING);
        } else if(indexer[2] == BallColors.GREEN){
            indexServo.home();
            indexer[2] = BallColors.NONE;
            breakBeamReads[2] = false;
            indexer[3] = BallColors.GREEN;
            breakBeamReads[3] = true;
            if (timer.seconds() > 0.5){
                setState(SHOOTING);
            }
        } else if (indexer[1] == BallColors.GREEN){
            rearIntake.setPower(1);
            frontIntake.setPower(0);
            if (timer.seconds() > 0.5){
                breakBeamReads[3] = true;
                indexer[3] = BallColors.GREEN;
                breakBeamReads[1] = false;
                indexer[1] = BallColors.NONE;
                setState(SHOOTING);
            }
        } else if (indexer[0] == BallColors.GREEN) {
            frontIntake.setPower(1);
            rearIntake.setPower(0);
            if (timer.seconds() > 0.5) {
                breakBeamReads[3] = true;
                indexer[3] = BallColors.GREEN;
                breakBeamReads[1] = false;
                indexer[1] = BallColors.NONE;
                setState(SHOOTING);
            }
        } else {
            setState(IDLE);
        }
    }

    public void loadPurple(){
        shooterDoor.closed();
        if (indexer[3] == BallColors.PURPLE){
            setState(SHOOTING);
        } else if(indexer[2] == BallColors.PURPLE){
            indexServo.home();
            indexer[2] = BallColors.NONE;
            breakBeamReads[2] = false;
            indexer[3] = BallColors.PURPLE;
            breakBeamReads[3] = true;
            if (timer.seconds() > 0.5){
                setState(SHOOTING);
            }
        } else if (indexer[1] == BallColors.PURPLE){
            rearIntake.setPower(1);
            frontIntake.setPower(0);
            if (timer.seconds() > 0.5){
                breakBeamReads[3] = true;
                indexer[3] = BallColors.PURPLE;
                breakBeamReads[1] = false;
                indexer[1] = BallColors.NONE;
                setState(SHOOTING);
            }
        } else if (indexer[0] == BallColors.PURPLE) {
            frontIntake.setPower(1);
            rearIntake.setPower(0);
            if (timer.seconds() > 0.5) {
                breakBeamReads[3] = true;
                indexer[3] = BallColors.PURPLE;
                breakBeamReads[1] = false;
                indexer[1] = BallColors.NONE;
                setState(SHOOTING);
            }
        } else {
            setState(IDLE);
        }
    }

    //This is for if we are in index mode and Avery chooses to shoot a ball without knowing its color
    //We need this because in index mode no ball is loaded into the shooting position so,
    //unlike in standard mode, we need to find a ball to shoot before we shoot it.
    public void loadAndShootUnindexed(){
        shooterDoor.closed();
        if (breakBeamReads[3]){
            setState(SHOOTING);
        } else if(breakBeamReads[2]){
            indexServo.home();
            indexer[3] = indexer[2];
            breakBeamReads[2] = false;
            indexer[2] = BallColors.NONE;
            breakBeamReads[3] = true;
            if (timer.seconds() > 0.5){
                setState(SHOOTING);
            }
        } else if (breakBeamReads[1]){
            rearIntake.setPower(1);
            frontIntake.setPower(0);
            if (timer.seconds() > 0.5){
                breakBeamReads[3] = true;
                indexer[3] = indexer[1];
                breakBeamReads[1] = false;
                indexer[1] = BallColors.NONE;
                setState(SHOOTING);
            }
        } else if (breakBeamReads[0]) {
            frontIntake.setPower(1);
            rearIntake.setPower(0);
            if (timer.seconds() > 0.5) {
                breakBeamReads[3] = true;
                indexer[3] = indexer[1];
                breakBeamReads[1] = false;
                indexer[1] = BallColors.NONE;
                setState(SHOOTING);
            }
        } else {
            setState(IDLE);
        }
    }

    //run both intakes the same direction (one in, one out), in order to pass a ball from the currently
    //active intake into the opposite side intake. When there is a ball in the opposite side intake, stop
    //that intake.
    public void intakeFront(){
        frontIntake.setPower(1);
        shooterDoor.closed();
        //if there is a ball in the rear intake, stop reversing that intake
        if(breakBeamReads[1]){
            rearIntake.setPower(0);
        } else {
            rearIntake.setPower(-0.5);
        }
    }
    public void intakeRear(){
        rearIntake.setPower(1);
        shooterDoor.closed();
        //if there is a ball in the front intake, stop reversing that intake
        if(breakBeamReads[0]){
            frontIntake.setPower(0);
        } else {
            frontIntake.setPower(-0.5);
        }

    }

    public void clearMagazine(){
        shooterDoor.closed();
        rearIntake.setPower(-1);
        frontIntake.setPower(-1);
        if (timer.seconds() > 0.2){
            indexServo.home();
        }
        breakBeamReads[0] = false;
        breakBeamReads[1] = false;
        breakBeamReads[2] = false;
        breakBeamReads[3] = false;
    }

    public void resetBreakBeams(){
        breakBeamReads[0] = false;
        breakBeamReads[1] = false;
        breakBeamReads[2] = false;
        breakBeamReads[3] = false;
    }


    @Override
    public void update() {
        work();
    }

    public void updateSensors(){
        for(int sensorNum = 0; sensorNum <= breakBeams.length; sensorNum++){
            //if we believe there is not a ball in one of the break beam slots, check if there is a ball
            //after we attempt to eject balls, we will set all the reads to false and read them again
            if(!breakBeamReads[sensorNum]){
                breakBeamReads[sensorNum] = breakBeams[sensorNum].isPressed();
            }
            //if there is a ball, and we do not know its color, and we are attempting to index, check the
            //color of the ball and record it to the indexer list
            if(breakBeamReads[sensorNum] && indexer[sensorNum] != BallColors.NONE && indexMode){
                NormalizedRGBA colors = colorSensors[sensorNum].getNormalizedColors();
                Color.colorToHSV(colors.toColor(),hsv);
                if (hsv[0] > 200){
                    indexer[sensorNum] = BallColors.PURPLE;
                } else if (hsv[0] > 100){
                    indexer[sensorNum] = BallColors.GREEN;
                }
            }
        }
    }

    /*@Override
    public void update() {
        work();
    }*/

    public enum IntakeMagazineStates{
        IDLE, INTAKEFRONT, INTAKEREAR, SHOOTING, LOAD, LOADGREEN, LOADPURPLE, CLEAR, LOADANDSHOOTUNINDEXED
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
}

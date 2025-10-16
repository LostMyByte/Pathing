package org.firstinspires.ftc.teamcode.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.Utilities.HardwareDevices.Servo;

public class Hopper extends Subsystem{

    CRServo spindexer;
    Servos.Transfer transfer;
    HopperStates hopperState;
    ElapsedTime timer = new ElapsedTime();
    public Hopper(HardwareMap hardwareMap) {
        hopperState = HopperStates.NOTACTIVE;
        spindexer = hardwareMap.get(CRServo.class, "spindexer");
        transfer = new Servos.Transfer();
    }
    @Config
    public static class HopperDash{
        public static double transferPos = 0.8;
        public static double downTransferPos = 1;
        //1 is all the way down, 0.8~ is transfer
        public static double timeToMoveOneSlot = 600; //milis
    }


    public enum HopperStates{
        MOVEBACKONE, MOVEONE, NOTACTIVE, TRANSFER, HOPPERSPINNNNNN; //hopperspinn is for testing
    }

    public Hopper.HopperStates getState(){
        return hopperState;
    }

    public void setState(Hopper.HopperStates state){
        hopperState = state;
    }
    public void work(){
        switch (hopperState){
            case NOTACTIVE:
               safePos();
                break;
            case TRANSFER:
                timer.reset();
                while (timer.milliseconds() < 100){
                transfer();}
                setState(HopperStates.NOTACTIVE);
                break;
            case MOVEBACKONE:
                timer.reset();
                while (timer.milliseconds() < HopperDash.timeToMoveOneSlot){
                moveOneSlot(true);}
                setState(HopperStates.NOTACTIVE);
                break;
            case MOVEONE:
                timer.reset();
                while (timer.milliseconds() < HopperDash.timeToMoveOneSlot){
                moveOneSlot(false);}
                setState(HopperStates.NOTACTIVE);
                break;
            case HOPPERSPINNNNNN:
                spindexer.setPower(1);
                break;

        }
    }

    public void transfer(){
        spindexer.setPower(0);
        transfer.setPosition(HopperDash.transferPos);
        //1 is all the way down, 0.8~ is transfer
    }

    private void moveOneSlot(boolean backwards){
        transfer.setPosition(1);
        if(backwards) {spindexer.setPower(-1);} else {spindexer.setPower(1); }
       spindexer.setPower(0);
    }
    public void safePos(){
        transfer.setPosition(HopperDash.downTransferPos);
        spindexer.setPower(0);
    }

    @Override
    public void updateSensors() {

    }

    @Override
    public void update() {

    }
}
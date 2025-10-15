package org.firstinspires.ftc.teamcode.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.teamcode.teamcode.Utilities.HardwareDevices.Servo;

public class Hopper extends Subsystem{

    CRServo spindexer;
    Servos.Transfer transfer;
    HopperStates hopperState;
    public Hopper(HardwareMap hardwareMap) {
        hopperState = HopperStates.TRANSFER;
        spindexer = hardwareMap.get(CRServo.class, "spindexer");
        transfer = new Servos.Transfer();
    }
    @Config
    public static class HopperDash{
        public static double transferPos = 1;   //1 is all the way down, 0.8~ is transfer
    }


    public enum HopperStates{
        MOVETOPOSITION, NOTACTIVE, TRANSFER, HOPPERSPINNNNNN; //hopperspinn is for testing
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
               // transfer.setPosition(0);
                spindexer.setPower(0);
                break;
            case TRANSFER:
                transfer();
                break;
            case MOVETOPOSITION:
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

    @Override
    public void updateSensors() {

    }

    @Override
    public void update() {

    }
}
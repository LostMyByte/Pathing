/*
package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.TeliOp.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Hopper;

@TeleOp(name="Indexer test", group="Iterative Opmode")
public class IndexerTest extends BaseOpMode {
    ElapsedTime timeWaste = new ElapsedTime();
    Boolean canTransfer = true;
    Boolean cross = false;

    @Override
    public void externalInit() {
        hopper = new Hopper(hardwareMap);
    }

    @Override
    public void externalLoop() {
        telemetry.update();
        // hopper.update();
        hopper.work();
        if (driver1.circle.isTapped() && canTransfer) {
            hopper.setState(Hopper.HopperStates.TRANSFER);
            BaseOpMode.addData("transfer", "transfer");
            if(hopper.getState() == Hopper.HopperStates.NOTACTIVE){
                canTransfer = true;
            }
        }else if (driver1.cross.isTapped()&&!cross) {
            hopper.setState(Hopper.HopperStates.HOPPERSPINNNNNN);
            canTransfer = false;
            BaseOpMode.addData("spinnnnn", "SPINNNNNNNNNNNNNNNNNNNNN");
            cross = true;
        }else if(driver1.cross.isTapped()&&cross){
            hopper.setState(Hopper.HopperStates.NOTACTIVE);
            cross = false;
            canTransfer = true;
        }else if(driver1.triangle.isTapped()){
            hopper.setState(Hopper.HopperStates.MOVEONE);
        }else if (driver1.square.isTapped()) {
            hopper.setState(Hopper.HopperStates.MOVEBACKONE);
        }else if (driver1.dpad_right.isTapped()){
            hopper.setState(Hopper.HopperStates.MOVEBACKTWO);
        }else if (driver1.dpad_left.isTapped()){
            hopper.setState(Hopper.HopperStates.MOVETWO);
        }else if (driver1.dpad_up.isTapped()){
            hopper.setState(Hopper.HopperStates.INTAKEPOSTOSHOOTPOS);
            canTransfer = !canTransfer;
        }else if (driver1.dpad_down.isTapped()){
            hopper.setState(Hopper.HopperStates.NOTACTIVE);
            canTransfer = !canTransfer;
        } else{
            ElapsedTime time1 = new ElapsedTime();
            if (time1.seconds()>1){
                hopper.setState(Hopper.HopperStates.NOTACTIVE);
                BaseOpMode.addData("nuh", "uh");
                time1.reset();
            }}
        BaseOpMode.addData("state", hopper.getState());
        BaseOpMode.addData("can transfer", canTransfer);
    }
}


 */

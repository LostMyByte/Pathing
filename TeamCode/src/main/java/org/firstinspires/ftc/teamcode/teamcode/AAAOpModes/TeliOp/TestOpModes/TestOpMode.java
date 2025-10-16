package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.TeliOp.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;

import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Hopper;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Dash.DashPositions;

@TeleOp(name="Test Teleop", group="Iterative Opmode")
public class TestOpMode extends BaseOpMode {


    Hopper hopper;
    ElapsedTime timeWaste = new ElapsedTime();
    Boolean circle = false;
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
        if (driver1.circle.isTapped() && !circle) {

            hopper.setState(Hopper.HopperStates.TRANSFER);
            circle = true;
            BaseOpMode.addData("transfer", "transfer");
            if(hopper.getState() == Hopper.HopperStates.NOTACTIVE){
                circle = false;
            }
        }else if (driver1.cross.isTapped()&&!cross) {
            hopper.setState(Hopper.HopperStates.HOPPERSPINNNNNN);
            BaseOpMode.addData("spinnnnn", "SPINNNNNNNNNNNNNNNNNNNNN");
            cross = true;
        }else if(driver1.cross.isTapped()&&cross){
            hopper.setState(Hopper.HopperStates.NOTACTIVE);
            cross = false;
        }else if(driver1.triangle.isTapped() && !circle){
            hopper.setState(Hopper.HopperStates.MOVEONE);
        }else if (driver1.square.isTapped() && !circle) {
            hopper.setState(Hopper.HopperStates.MOVEBACKONE);
        }else{
            ElapsedTime time1 = new ElapsedTime();
            if (time1.seconds()>1){
            hopper.setState(Hopper.HopperStates.NOTACTIVE);
            BaseOpMode.addData("nuh", "uh");
            time1.reset();
        }}
    }
}
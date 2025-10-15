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


    @Override
    public void externalInit() {

        hopper = new Hopper(hardwareMap);

    }

    @Override
    public void externalLoop() {
       telemetry.update();
      // hopper.update();
        hopper.work();
       hopper.setState(Hopper.HopperStates.TRANSFER);
    }
}
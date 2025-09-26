package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.TeliOp.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;

import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Shooter;

@TeleOp(name="Test Teleop", group="Iterative Opmode")
public class TestOpMode extends BaseOpMode {


    Shooter shooter;

    @Override
    public void externalInit() {
        shooter = new Shooter(hardwareMap, 0, Constants.Team.BLUE);
    }

    @Override
    public void externalLoop() {
        shooter.setState(Shooter.ShooterStates.SHOOTERTESTING);

    }
}
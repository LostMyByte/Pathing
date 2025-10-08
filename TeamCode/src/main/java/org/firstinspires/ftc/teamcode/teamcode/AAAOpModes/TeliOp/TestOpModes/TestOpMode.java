package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.TeliOp.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;

import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Dash.DashPositions;

@TeleOp(name="Test Teleop", group="Iterative Opmode")
public class TestOpMode extends BaseOpMode {


    Shooter shooter;
    ElapsedTime timer;

    @Override
    public void externalInit() {

        shooter = new Shooter(hardwareMap, 0, Constants.Team.BLUE);
        timer = new ElapsedTime();
        timer.reset();
    }

    @Override
    public void externalLoop() {
        shooter.setState(Shooter.ShooterStates.SHOOTERTESTING);

        DashPositions.dashShooterRPM = 2100;


        BaseOpMode.addData("time", timer.seconds());

        if (timer.seconds() < 10){
            shooter.setHoodAngleBasedOnTargetShotAngle(25);
        }
        if (timer.seconds() > 10){
            shooter.setHoodAngleBasedOnTargetShotAngle(65);
        }
    }
}
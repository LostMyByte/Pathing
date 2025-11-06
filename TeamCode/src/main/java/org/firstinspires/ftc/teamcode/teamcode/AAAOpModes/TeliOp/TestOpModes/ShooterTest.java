// Primary Author: Dylan Cook
package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.TeliOp.TestOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Constants;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Servos;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Dash.ShooterDashClass;
import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
@TeleOp (name = "ShooterTest")
public class ShooterTest extends BaseOpMode {

Shooter shooter;
Servos.Hood hood;

    double ticksL = 0;
    double ticksR = 0;
    @Config
    public static class ShooterDash{
        public static double ticksPerRotation = 28*(3.0/2); //This is now ticks per revolution
    }

    @Override
    public void externalInit() {
        shooter = new Shooter(hardwareMap, 0, Constants.Team.BLUE);
        hood = new Servos.Hood();
    }


    @Override
    public void externalLoop() {
        shooter.setState(Shooter.ShooterStates.ACTIVE);
        shooter.setTargetShooterRPM(ShooterDashClass.shooterSpeed);
        BaseOpMode.addData("targetRPM", shooter.getTargetShooterRPM());
        BaseOpMode.addData("RPM", shooter.getShooterRPM());
    }
}

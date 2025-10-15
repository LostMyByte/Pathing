package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.TeliOp.TestOpModes;

import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.TeliOp.TestOpModes.ShooterTest.ShooterDash.ticksPerRotation;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Servos;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Dash.DashPositions;

@TeleOp (name = "ShooterTest")
public class ShooterTest extends BaseOpMode {

Shooter shooter;
Servos.Hood hood;

    double ticksL = 0;
    double ticksR = 0;
    @Config
    public static class ShooterDash{
        public static double shooterMotorPower = 0;
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
        shooter.setTargetShooterRPM(1950);
        BaseOpMode.addData("targetRPM", shooter.getTargetShooterRPM());
        BaseOpMode.addData("RPM", shooter.getShooterRPM());
    }
}

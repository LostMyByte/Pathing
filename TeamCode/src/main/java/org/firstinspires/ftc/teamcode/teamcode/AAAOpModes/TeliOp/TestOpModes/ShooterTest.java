package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.TeliOp.TestOpModes;

import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.TeliOp.TestOpModes.ShooterTest.ShooterDash.ticksPerRotation;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;

@TeleOp (name = "ShooterTest")
public class ShooterTest extends BaseOpMode {

    DcMotorEx leftMotorEncoder;
    DcMotorEx rightMotorEncoder;
    //named left and right as if behind shooter such that you would not be hit by projectiles being shot

    ElapsedTime timer;

    double ticksL = 0;
    double ticksR = 0;
    @Config
    public static class ShooterDash{
        public static double shooterMotorPower = 0;
        public static double ticksPerRotation = 28*(3.0/2); //This is now ticks per revolution
    }

    @Override
    public void externalInit() {
        leftMotorEncoder = hardwareMap.get(DcMotorEx.class, "left");
        rightMotorEncoder = hardwareMap.get(DcMotorEx.class, "right");
        timer = new ElapsedTime();
        timer.reset();


    }


    @Override
    public void externalLoop() {


        //ticksL = leftMotorEncoder.getCurrentPosition()-ticksL;
        //ticksR = rightMotorEncoder.getCurrentPosition() - ticksR;


        multTelemetry.addData("left motor RPM", leftMotorEncoder.getVelocity()/ShooterDash.ticksPerRotation*60);
        multTelemetry.addData("right motor RPM", rightMotorEncoder.getVelocity()/ShooterDash.ticksPerRotation*60);
      // multTelemetry.addData("left motor RPM", (ticksL/ticksPerRotation)/(timer.milliseconds())*1000*60);
       //multTelemetry.addData("right motor RPM", (ticksR/ticksPerRotation)/(timer.milliseconds())*1000*60);
       //multTelemetry.addData("ticks since refresh", ticksL);
       timer.reset();


    }
}

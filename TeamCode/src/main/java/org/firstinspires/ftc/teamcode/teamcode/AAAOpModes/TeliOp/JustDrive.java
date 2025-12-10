package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.TeliOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains.TankDriveTrain;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Localization.Location;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.TeliOpDrivetrain;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;


// A basic teleop for driving around and doing some testing
@TeleOp(name = "Just Tank Drive")
public class JustDrive extends BaseOpMode {

    TeliOpDrivetrain drive;

    @Override
    public void externalInit () {
        drive = new TeliOpDrivetrain(hardware, 0);
    }

    @Override
    public void externalLoop () {
        if (driver1.rightStick.isPressed()){
            drive.PIDdrive(driver1.leftStick.Y(), -driver1.rightStick.X(), 0.3);
        } else {
            drive.PIDdrive(driver1.leftStick.Y(), -driver1.rightStick.X(), 1);
        }
    }
}

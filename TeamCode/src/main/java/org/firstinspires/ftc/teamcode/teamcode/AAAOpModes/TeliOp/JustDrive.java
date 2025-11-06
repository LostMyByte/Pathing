package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.TeliOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains.TankDriveTrain;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Localization.Location;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;


// A basic teleop for driving around and doing some testing
@TeleOp(name = "Just Tank Drive")
public class JustDrive extends BaseOpMode {

    TankDriveTrain drive;

    Location loc;
    @Override
    public void externalInit () {
        loc = new Location(0,0,0);
        drive = new TankDriveTrain();
    }

    @Override
    public void externalLoop () {

        // Update Odometry Pod offsets -- used for tuning
        loc.updateOffsets();
        Vector target = new Vector(-gamepad1.left_stick_y, gamepad1.right_stick_x);

        drive.move(target);
    }
}

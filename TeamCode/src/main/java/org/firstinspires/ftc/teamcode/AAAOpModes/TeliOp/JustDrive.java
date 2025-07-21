package org.firstinspires.ftc.teamcode.AAAOpModes.TeliOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.Motion.Drivetrains.FixedDriveTrain;
import org.firstinspires.ftc.teamcode.Utilities.Configuration.DriveConfig;
import org.firstinspires.ftc.teamcode.Utilities.LinearAlgebra.Vector;

@TeleOp(name = "Just Drive")
public class JustDrive extends BaseOpMode {

    FixedDriveTrain drive;

    @Override
    public void externalInit () {
        drive = new FixedDriveTrain();
    }

    @Override
    public void externalLoop () {
        Vector target = (new Vector(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x));

        if (gamepad1.dpad_down)     target.add(new Vector (0, -0.5, 0));;
        if (gamepad1.dpad_up)       target.add(new Vector (0, 0.5, 0));
        if (gamepad1.dpad_right)    target.add(new Vector (0.5, 0, 0));
        if (gamepad1.dpad_left)     target.add(new Vector (-0.5, 0, 0));

        BaseOpMode.addData("Drive Vector X", target.get(0));
        BaseOpMode.addData("Drive Vector Y", target.get(1));
        BaseOpMode.addData("Drive Vector H", target.get(2));

        if (gamepad1.square) {
            BaseOpMode.addLine("Adjusted Drivewheel Powers");
            DriveConfig.driveWheels[0].MovementVector = new Vector(DriveConfig.DriveWheels.FR.x, DriveConfig.DriveWheels.FR.y, DriveConfig.DriveWheels.FR.h);
            DriveConfig.driveWheels[1].MovementVector = new Vector(DriveConfig.DriveWheels.FL.x, DriveConfig.DriveWheels.FL.y, DriveConfig.DriveWheels.FL.h);
            DriveConfig.driveWheels[2].MovementVector = new Vector(DriveConfig.DriveWheels.BR.x, DriveConfig.DriveWheels.BR.y, DriveConfig.DriveWheels.BR.h);
            DriveConfig.driveWheels[3].MovementVector = new Vector(DriveConfig.DriveWheels.BL.x, DriveConfig.DriveWheels.BL.y, DriveConfig.DriveWheels.BL.h);

        }

        drive.move(target);
    }
}

package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.TeliOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains.TankDriveTrain;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;

@TeleOp(name = "Just Tank Drive")
public class JustDrive extends BaseOpMode {

    TankDriveTrain drive;

    @Override
    public void externalInit () {
        drive = new TankDriveTrain(new Vector(0,0,0));
    }

    @Override
    public void externalLoop () {

        drive.loc.updateOffsets();
        Vector target = (new Vector(-gamepad1.left_stick_y, gamepad1.right_stick_x));



        if (gamepad1.square) {
            BaseOpMode.addLine("Adjusted Drivewheel Powers");
            /*Movement.driveWheels[0].MovementVector = new Vector(DriveConfig.DriveWheels.FR.x, DriveConfig.DriveWheels.FR.y, DriveConfig.DriveWheels.FR.h);
            DriveConfig.driveWheels[1].MovementVector = new Vector(DriveConfig.DriveWheels.FL.x, DriveConfig.DriveWheels.FL.y, DriveConfig.DriveWheels.FL.h);
            DriveConfig.driveWheels[2].MovementVector = new Vector(DriveConfig.DriveWheels.BR.x, DriveConfig.DriveWheels.BR.y, DriveConfig.DriveWheels.BR.h);
            DriveConfig.driveWheels[3].MovementVector = new Vector(DriveConfig.DriveWheels.BL.x, DriveConfig.DriveWheels.BL.y, DriveConfig.DriveWheels.BL.h);*/

        }

        drive.move(target);
    }
}

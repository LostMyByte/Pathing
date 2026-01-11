package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.TeliOp;

import static org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains.SystemModels.MechanumDrive.W;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains.FixedDriveTrain;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains.SystemModels.MechanumDrive;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains.TankDriveTrain;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Localization.Location;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.TeliOpDrivetrain;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.DriveWheels;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.GeneralMatrix;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Matrix;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;


// A basic teleop for driving around and doing some testing
@TeleOp(name = "Just Tank Drive")
public class JustDrive extends BaseOpMode {

    FixedDriveTrain drive;
    Location loc;

    @Override
    public void externalInit () {
        loc = new Location(0,0,0);
        drive = new FixedDriveTrain(loc); //new TeliOpDrivetrain(hardware, 0);
        loc.doTelemetry = true;
    }

    @Override
    public void externalLoop () {
        //drive.updateOdo(loc.getPosX(), loc.getPosY(), loc.getPosH());
        W = new GeneralMatrix(4, 3, new double[] {
                DriveWheels.FR.y, DriveWheels.FR.x, DriveWheels.FR.h,
                DriveWheels.FL.y, DriveWheels.FL.x, DriveWheels.FL.h,
                DriveWheels.BR.y, DriveWheels.BR.x, DriveWheels.BR.h,
                DriveWheels.BL.y, DriveWheels.BL.x, DriveWheels.BL.h,
        });
        MechanumDrive.WL = W.transposed().multiplied(0.25);
        if (driver1.rightStick.isPressed()){
            drive.move(driver1.rightStick.Y(), driver1.rightStick.X(), driver1.leftStick.X(), 0.3);
        } else {
            drive.move(driver1.rightStick.Y(), driver1.rightStick.X(), driver1.leftStick.X(), 1);
        }
    }
}

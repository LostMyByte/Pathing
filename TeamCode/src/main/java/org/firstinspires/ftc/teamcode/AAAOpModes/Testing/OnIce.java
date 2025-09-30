package org.firstinspires.ftc.teamcode.AAAOpModes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.Motion.Drivetrains.TankDriveTrain;
import org.firstinspires.ftc.teamcode.Motion.Localization.Location;
import org.firstinspires.ftc.teamcode.Motion.SystemModels.TankDrive;
import org.firstinspires.ftc.teamcode.Utilities.Configuration.DriveConfig;
import org.firstinspires.ftc.teamcode.Utilities.Math.Vector;

@TeleOp(name = "Wolfpack on Ice")
public class OnIce extends BaseOpMode {

    private TankDriveTrain drive;

    private Location sensorSignal;

    @Override
    public void externalInit() {
        drive = new TankDriveTrain(new Vector(0, 0, 0));
        sensorSignal = drive.loc;

    }

    @Override
    public void externalLoop() {

        Vector sensorData = sensorSignal.getPositionForTankDrive();
        Vector target = Vector.length(5);

        Vector additional = new Vector(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x).multiplied(DriveConfig.DriveWheels.driveAcceleration);

        for (int i = 3; i < 5; i++) {
            target.put(i, sensorData.get(i));
        }


        Vector powers = TankDrive.getBLeftInverse(sensorData).multiplied(target.subtracted(TankDrive.getAMatrix(sensorData).multiplied(sensorData)));

        BaseOpMode.addData("Correction L", powers.get(0));
        BaseOpMode.addData("Correction R", powers.get(1));
        drive.moveRaw(powers);
        TankDrive.reInit();

    }
}

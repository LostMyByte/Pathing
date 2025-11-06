// Primary Author: Kieran Mattingly
package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains.TankDriveTrain;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Localization.Location;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains.SystemModels.TankDrive;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.DriveWheels;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;

@TeleOp(name = "Wolfpack on Ice")
public class OnIce extends BaseOpMode {

    private TankDriveTrain drive;

    private Location sensorSignal;

    TankDrive model;

    @Override
    public void externalInit() {
        drive = new TankDriveTrain();
        sensorSignal = new Location(0, 0, 0);
        model = new TankDrive();

    }

    @Override
    public void externalLoop() {

        model = new TankDrive();

        Vector sensorData = sensorSignal.getPositionForTankDrive();
        Vector target = Vector.length(5);

        Vector additional = new Vector(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x).multiplied(DriveWheels.driveAcceleration);

        for (int i = 3; i < 5; i++) {
            target.put(i, sensorData.get(i));
        }


        Vector powers = TankDrive.getBLeftInverse(sensorData).multiplied(target.subtracted(model.getAMatrix(sensorData).multiplied(sensorData)));

        BaseOpMode.addData("Correction L", powers.get(0));
        BaseOpMode.addData("Correction R", powers.get(1));
        drive.moveRaw(powers);


    }
}

package org.firstinspires.ftc.teamcode.AAAOpModes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.Motion.Controllers.Signal;
import org.firstinspires.ftc.teamcode.Motion.DriveModel;
import org.firstinspires.ftc.teamcode.Motion.Drivetrains.FixedDriveTrain;
import org.firstinspires.ftc.teamcode.Utilities.Configuration.DriveConfig;
import org.firstinspires.ftc.teamcode.Utilities.Math.Vector;

@TeleOp(name = "Wolfpack on Ice")
public class OnIce extends BaseOpMode {

    private FixedDriveTrain drive;

    private Signal sensorSignal;

    @Override
    public void externalInit() {
        drive = new FixedDriveTrain(new Vector(0, 0, 0));
        sensorSignal = drive.loc;

    }

    @Override
    public void externalLoop() {
        int numDim = this.sensorSignal.getLength();
        Vector sensorData = Vector.length(numDim * 2);
        Vector target = Vector.length(6);

        Vector additional = new Vector(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x).multiplied(DriveConfig.DriveWheels.driveAcceleration);

        for (int i = 0; i < numDim; i++) {
            sensorData.put(i, sensorSignal.getIntegralVector().get(i));
            sensorData.put(i+numDim, sensorSignal.getDataVector().get(i) + additional.get(i));
            target.put(i+3, sensorSignal.getDataVector().get(i));
        }



        Vector powers = DriveModel.getBLeftInverse(sensorData).multiplied(target.subtracted(DriveModel.getAMatrix(sensorData).multiplied(sensorData)));

        BaseOpMode.addData("Correcction X", powers.get(0));
        BaseOpMode.addData("Correcction Y", powers.get(1));
        BaseOpMode.addData("Correcction H", powers.get(2));
        drive.move(powers);
        DriveModel.reInit();

    }
}

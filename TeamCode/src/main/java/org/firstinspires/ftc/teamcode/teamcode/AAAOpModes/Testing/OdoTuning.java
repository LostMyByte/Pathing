package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Testing;

import static org.firstinspires.ftc.teamcode.teamcode.Motion.Localization.Location.xOffset;
import static org.firstinspires.ftc.teamcode.teamcode.Motion.Localization.Location.yOffset;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains.FixedDriveTrain;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Localization.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Movement;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;


@TeleOp(name = "OdoTuning")
public class OdoTuning extends BaseOpMode {



    Movement drive;
    @Override
    public void externalInit() {
        drive = new FixedDriveTrain(new Vector(0, 0, 0));
    }
    double oldEX = 0;
    double oldEY = 0;
    @Override
    public void externalLoop() {

        drive.move(new Vector(0, 0, gamepad1.right_stick_x));

        drive.loc.odoPods.setOffsets(xOffset,yOffset);

        addData("Encoder X", drive.loc.odoPods.getEncoderX());
        addData("Encoder Y", drive.loc.odoPods.getEncoderY());

        if (gamepad1.square) {
            drive.loc.setPosition(new Vector(0, 0,0));
            oldEX = drive.loc.odoPods.getEncoderX();
            oldEY = drive.loc.odoPods.getEncoderX();
        }

        double xDist = (drive.loc.odoPods.getEncoderX() - oldEX)/GoBildaPinpointDriver.goBILDA_4_BAR_POD;
        double yDist = (drive.loc.odoPods.getEncoderY() - oldEY)/GoBildaPinpointDriver.goBILDA_4_BAR_POD;
        double angle = drive.loc.getPosition().get(2);

        addData("X Offset", xDist/angle);
        addData("Y Offset", yDist/angle);

    }

}

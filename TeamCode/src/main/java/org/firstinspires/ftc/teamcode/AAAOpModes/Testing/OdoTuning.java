package org.firstinspires.ftc.teamcode.AAAOpModes.Testing;

import static org.firstinspires.ftc.teamcode.Motion.Localization.Location.xOffset;
import static org.firstinspires.ftc.teamcode.Motion.Localization.Location.yOffset;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.Motion.Drivetrains.FixedDriveTrain;
import org.firstinspires.ftc.teamcode.Motion.Localization.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Motion.Movement;
import org.firstinspires.ftc.teamcode.Utilities.Configuration.Hardware;
import org.firstinspires.ftc.teamcode.Utilities.Math.Vector;


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

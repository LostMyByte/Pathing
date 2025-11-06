// Primary Author: Kieran Mattingly
package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Testing;

import static org.firstinspires.ftc.teamcode.teamcode.Motion.Localization.Location.xOffset;
import static org.firstinspires.ftc.teamcode.teamcode.Motion.Localization.Location.yOffset;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains.FixedDriveTrain;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Localization.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains.Movement;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Localization.Location;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;


@TeleOp(name = "OdoTuning")
public class OdoTuning extends BaseOpMode {



    Movement drive;
    Location loc;
    @Override
    public void externalInit() {
        loc = new Location(0, 0, 0);
        drive = new FixedDriveTrain(loc);
    }
    double oldEX = 0;
    double oldEY = 0;
    @Override
    public void externalLoop() {

        drive.move(new Vector(0, 0, gamepad1.right_stick_x));

        loc.odoPods.setOffsets(xOffset,yOffset);

        addData("Encoder X", loc.odoPods.getEncoderX());
        addData("Encoder Y", loc.odoPods.getEncoderY());

        if (gamepad1.square) {
            loc.setPosition(new Vector(0, 0,0));
            oldEX = loc.odoPods.getEncoderX();
            oldEY = loc.odoPods.getEncoderX();
        }

        double xDist = (loc.odoPods.getEncoderX() - oldEX)/GoBildaPinpointDriver.goBILDA_4_BAR_POD;
        double yDist = (loc.odoPods.getEncoderY() - oldEY)/GoBildaPinpointDriver.goBILDA_4_BAR_POD;
        double angle = loc.getPosition().get(2);

        addData("X Offset", xDist/angle);
        addData("Y Offset", yDist/angle);

    }

}

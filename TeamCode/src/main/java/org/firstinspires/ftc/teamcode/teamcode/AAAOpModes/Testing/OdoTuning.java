// Primary Author: Kieran Mattingly
package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains.FixedDriveTrain;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains.TankDriveTrain;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Localization.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains.Movement;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Localization.OdoPodData;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;


@TeleOp(name = "OdoTuning")
public class OdoTuning extends BaseOpMode {



    TankDriveTrain drive;
    OdoPodData odos;
    @Override
    public void externalInit() {
        odos = new OdoPodData(0, 0, 0);
        odos.doTelemetry = true;
        drive = new TankDriveTrain();

    }
    double oldEX = 0;
    double oldEY = 0;
    @Override
    public void externalLoop() {


        drive.drive(-gamepad1.left_stick_y, gamepad1.right_stick_x);

        odos.updateOffsets();

        if (gamepad1.square) {
            odos.setPosition(0,0,0);
            oldEX = odos.odoPods.getEncoderX();
            oldEY = odos.odoPods.getEncoderX();
        }

        double xDist = (odos.odoPods.getEncoderX() - oldEX)/GoBildaPinpointDriver.goBILDA_4_BAR_POD;
        double yDist = (odos.odoPods.getEncoderY() - oldEY)/GoBildaPinpointDriver.goBILDA_4_BAR_POD;
        double angle = odos.getDataVector().get(2);

        addData("X Offset", xDist/angle);
        addData("Y Offset", yDist/angle);

    }

}

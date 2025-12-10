package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Localization.Location;

@TeleOp(name = "EKF Testing")
public class KalmanTesting extends BaseOpMode {

    Location loc;

    @Override
    public void externalInit() {
        loc = new Location(0, 0, 0);
        loc.doTelemetry = true;
    }

    @Override
    public void externalLoop() {
        loc.updateOdoOffsets();

    }
}

package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Localization.Location;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Constants;

@TeleOp(name = "Localization Testing")
public class KalmanTesting extends BaseOpMode {

    Location loc;

    @Override
    public void externalInit() {
        Constants.team = Constants.Team.RED;
        loc = new Location(182.88, 182.88, Math.PI/2);
        loc.doTelemetry = true;
    }

    @Override
    public void externalLoop() {
        loc.updateOdoOffsets();

    }
}

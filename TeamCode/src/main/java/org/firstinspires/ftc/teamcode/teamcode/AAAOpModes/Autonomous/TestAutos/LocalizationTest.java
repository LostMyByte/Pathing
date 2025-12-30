package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.TestAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Localization.Location;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Constants;

@Autonomous
public class LocalizationTest extends BaseOpMode{

    Location loc;

    @Override
    public void externalInit() {
        Constants.team = Constants.Team.RED;
        loc = new Location(140,341,Math.PI);
    }

    @Override
    public void externalLoop(){
        loc.doTelemetry = true;
    }
}

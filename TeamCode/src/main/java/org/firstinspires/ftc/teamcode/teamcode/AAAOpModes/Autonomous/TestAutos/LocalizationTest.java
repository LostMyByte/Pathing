package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.TestAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Localization.Location;

@Autonomous
public class LocalizationTest extends BaseOpMode{

    Location loc;

    @Override
    public void externalInit() {
        loc = new Location(120,340,3*Math.PI/4);
    }

    @Override
    public void externalLoop(){
        loc.doTelemetry = true;
    }
}

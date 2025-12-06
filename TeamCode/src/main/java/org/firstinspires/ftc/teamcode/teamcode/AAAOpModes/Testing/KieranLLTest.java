package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Localization.LimeLightData;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Localization.Location;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Localization.OdoPodData;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.BadTurretForTesting;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Constants;

@TeleOp(name = "Kieran LimeLightTesting")
public class KieranLLTest extends BaseOpMode {

    @Config
    public static class LLTestParams {
        public static boolean update = false;
        public static boolean invertFrac = false;
        public static double hScale = -1;
        public static double piScale = -0.5;

    }

    BadTurretForTesting turret;
    Location loc;
    @Override
    public void externalInit() {
        Constants.team = Constants.Team.BLUE;
        turret = new BadTurretForTesting();
        loc = new Location(-48*2.54, 48*2.54, -Math.toRadians(135));
        loc.doTelemetry = true;
    }

    @Override
    public void externalLoop() {

        if (gamepad1.circle || LLTestParams.update) {
            loc.setPositionToLL();
        }

        turret.setAngle( Math.atan(LLTestParams.invertFrac ? loc.getPosX()/loc.getPosY() : loc.getPosY()/loc.getPosX()) + LLTestParams.hScale * loc.getPosH() +LLTestParams.piScale *Math.PI);

    }
}

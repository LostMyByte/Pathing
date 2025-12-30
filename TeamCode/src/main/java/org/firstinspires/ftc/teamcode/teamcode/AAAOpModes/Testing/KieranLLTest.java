package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Testing;

import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Constants.Team.BLUE;
import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Constants.team;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Localization.LimeLightData;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Localization.Location;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Localization.OdoPodData;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.BadTurretForTesting;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Constants;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;

@TeleOp(name = "Kieran LimeLightTesting")
public class KieranLLTest extends BaseOpMode {

    @Config
    public static class LLTestParams {
        public static boolean update = false;
        public static boolean invertFrac = false;
        public static double hScale = -1;
        public static double piScale = -0.5;
        public static int speed = 1500;
    }

    Shooter ballGun;
    Location loc;

    Vector lastpos = new Vector(0,0);
    Vector lastOdo = new Vector(0,0);
    double lasth = 0;
    double lastturret = 0;
    @Override
    public void externalInit() {
        Constants.team = Constants.Team.RED;

        ballGun = new Shooter(hardware, Constants.Team.RED);
        ballGun.setState(Shooter.ShooterStates.ACTIVE);
        ballGun.setTargetShooterRPM(0);
        Location.llAlpha = 0;
        loc = new Location(183, 183, Math.toRadians(90));
        loc.doTelemetry = true;
    }

    @Override
    public void externalLoop() {

        ballGun.recieveOdoInputs(loc.getPosX(), loc.getPosY(), loc.getWrappedAngle(), loc.getTranslationalVelocity());

        if (gamepad1.circle || LLTestParams.update) {
            //loc.setPositionToLL();
        }

        if (gamepad1.square) {
            lastpos = LimeLightData.rawData;
            lastturret = LimeLightData.turretAngle;
            lastOdo = new Vector(loc.getPosX(), loc.getPosY());
            lasth = loc.getPosH();
        }

        Vector test = new Vector(lastpos.get(0), lastpos.get(1)).added(new Vector(loc.getPosX(), loc.getPosY()).subtracted(lastOdo));
        test = LimeLightData.handleOffsets(test, lasth, lastturret);
        addData("Last LL X", test.get(0));
        addData("Last LL Y", test.get(1));
        addData("Last LL H", lasth);



    }
}

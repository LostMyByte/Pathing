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

    BadTurretForTesting turret;
    Shooter ballGun;
    Location loc;
    @Override
    public void externalInit() {
        Constants.team = Constants.Team.RED;
        turret = new BadTurretForTesting();
        //ballGun = new Shooter(hardware, 0, Constants.Team.RED);
        //ballGun.setState(Shooter.ShooterStates.ACTIVE);
        //ballGun.setTargetShooterRPM(LLTestParams.speed);
        loc = new Location(48*2.54, 48*2.54, Math.toRadians(135));
        loc.doTelemetry = true;
    }

    @Override
    public void externalLoop() {

        if (gamepad1.circle || LLTestParams.update) {
            loc.setPositionToLL();
        }

        turret.setAngle( Math.atan(LLTestParams.invertFrac ? loc.getPosX()/loc.getPosY() : loc.getPosY()/loc.getPosX()) + LLTestParams.hScale * loc.getPosH() + (team ==BLUE ? 1 : -1) * LLTestParams.piScale *Math.PI);
        //ballGun.recieveOdoInputs(loc.getPosX()/100, loc.getPosY()/100, loc.getPosH(), loc.getTranslationalVelocity().multiplied(0.01));

    }
}

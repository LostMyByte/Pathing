package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.TeliOp.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.KCP.Localization.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.teamcode.KCP.Localization.Location;
import org.firstinspires.ftc.teamcode.teamcode.KCP.Localization.TwoWheelOdometry;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.zLibraries.Utilities.Vector2d;

@TeleOp
public class OdoBasedTurretAimTeleop extends BaseOpMode {

    TwoWheelOdometry odo;
    Shooter shooter;

    @Override
    public void externalInit() {
        odo = new TwoWheelOdometry(.15, 1.85, 0);
        shooter = new Shooter(hardwareMap, 0, Constants.Team.RED);
        shooter.setState(Shooter.ShooterStates.OBELISK);
    }

    @Override
    public void externalLoop() {
        odo.localize();
        Vector2d velocityVector = new Vector2d(odo.getVelocity()[0], odo.getVelocity()[1]);
        shooter.setTargetShooterRPM(1500);
        shooter.getTargetTurretAngle(Location.x(), Location.y(), Location.heading(), velocityVector);
        BaseOpMode.addData("x",Location.x());
        BaseOpMode.addData("y",Location.y());
        BaseOpMode.addData("h",Location.heading());
    }
}

package org.firstinspires.ftc.teamcode.AAAOpModes.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.Motion.Drivetrains.FixedDriveTrain;
import org.firstinspires.ftc.teamcode.Motion.Movement;
import org.firstinspires.ftc.teamcode.Motion.Paths.BezierPath;
import org.firstinspires.ftc.teamcode.Utilities.Math.Vector;

@TeleOp(name = "Bezier Curve Test", group = "Testing")
public class BezierCurveTest extends BaseOpMode {
    Movement drive;

    @Config
    public static class Pathsetup {
        public static double startVX = 0;
        public static double startVY = 0;
        public static double startVH = 0;

        public static double targetX = 0;
        public static double targetY = 0;
        public static double targetH = 0;

        public static double targetVX = 0;
        public static double targetVY = 0;
        public static double targetVH = 0;
        public static double speed = 0.9;
    }

    @Override
    public void externalInit() {
        drive = new FixedDriveTrain(new Vector(0,0,0));
    }

    @Override
    public void externalLoop() {
        if (gamepad1.square) {
            Vector start = drive.loc.getPosition();
            Vector end = new Vector(Pathsetup.targetX, Pathsetup.targetY, Pathsetup.targetH);
            Vector endV = new Vector(Pathsetup.targetVX, Pathsetup.targetVY, Pathsetup.targetVH);
            Vector startV = drive.loc.getDataVector();
            drive.followPath(new BezierPath(start, end, startV, endV), Pathsetup.speed);
        }
    }
}

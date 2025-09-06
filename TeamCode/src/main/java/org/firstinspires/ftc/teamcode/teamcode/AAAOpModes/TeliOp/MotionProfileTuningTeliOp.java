package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.TeliOp;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.KCP.Movement;

@TeleOp(name = "Motion Profiling")
public class MotionProfileTuningTeliOp extends BaseOpMode {

    Movement drivetrain;

    FtcDashboard dash;

    @Config
    public static class ProfilingPoints {
        public static double x1 = 0;
        public static double y1 = 0;
        public static double h1 = 0;
        public static double x2 = 1;
        public static double y2 = 0;
        public static double h2 = 0;
        public static double x3 = 1;
        public static double y3 = 100;
        public static double h3 = 0;
        public static double x4 = 0;
        public static double y4 = 1;
        public static double h4 = 0;
    }

    enum States {
        Point1,
        Point2,
        Point3,
        Point4
    }

    States state;

    boolean manual = true;

    @Override
    public void externalInit() {
        dash = FtcDashboard.getInstance();
        drivetrain = new Movement(0, 0, 0);
        state = States.Point1;
    }

    @Override
    public void externalLoop() {

        double x = drivetrain.getX();
        double y = drivetrain.getY();
        double h = drivetrain.getHeading();

        double[] velocity = drivetrain.getVelocity();

        drivetrain.setActiveCorrectionMethod(Movement.CorrectionMethods.Profiled);


        BaseOpMode.addData("X", x);
        BaseOpMode.addData("Y", y);
        BaseOpMode.addData("H (deg)", h * 180/Math.PI);

        TelemetryPacket packet = new TelemetryPacket(false);
        packet.fieldOverlay()
                .setFill("gray")
                .setStroke("blue")
                .strokeCircle(x, y, 1)
                .strokeLine(x, y, x + Math.cos(h), y+ Math.sin(h))
                .setStroke("green")
                .strokeLine(x, y, x+velocity[0], y+velocity[0]);


        dash.sendTelemetryPacket(packet);

        if (driver1.square.isTapped()) {
            manual = !manual;
        } else if (driver1.dpad_right.isTapped()) {
            state = States.Point1;
        } else if (driver1.dpad_up.isTapped()) {
            state = States.Point2;
        } else if (driver1.dpad_left.isTapped()) {
            state = States.Point3;
        } else if (driver1.dpad_down.isTapped()) {
            state = States.Point4;
        }

        stateMachine();
    }

    public void stateMachine() {
        switch (state) {
            case Point1:
                point1();
                break;
            case Point2:
                point2();
                break;
            case Point3:
                point3();
                break;
            case Point4:
                point4();
                break;
        }
    }

    public void point1() {
        if (drivetrain.holdPosition(ProfilingPoints.x1,ProfilingPoints.y1, ProfilingPoints.h1) && !manual) state = States.Point2;
    }
    public void point2() {
        if (drivetrain.holdPosition(ProfilingPoints.x2,ProfilingPoints.y2, ProfilingPoints.h2) && !manual) state = States.Point3;
    }
    public void point3() {
        if (drivetrain.holdPosition(ProfilingPoints.x3,ProfilingPoints.y3, ProfilingPoints.h3) && !manual) state = States.Point4;
    }
    public void point4() {
        if (drivetrain.holdPosition(ProfilingPoints.x4,ProfilingPoints.y4, ProfilingPoints.h4) && !manual) state = States.Point1;
    }
}

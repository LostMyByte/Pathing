package org.firstinspires.ftc.teamcode.AAAOpModes.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.Motion.Controllers.ConstantSignal;
import org.firstinspires.ftc.teamcode.Motion.Controllers.MPC;
import org.firstinspires.ftc.teamcode.Motion.Controllers.MPCPath;
import org.firstinspires.ftc.teamcode.Motion.Controllers.Signal;
import org.firstinspires.ftc.teamcode.Motion.DriveModel;
import org.firstinspires.ftc.teamcode.Motion.Drivetrains.FixedDriveTrain;
import org.firstinspires.ftc.teamcode.Motion.Localization.Location;
import org.firstinspires.ftc.teamcode.Motion.Movement;
import org.firstinspires.ftc.teamcode.Utilities.Configuration.DriveConfig;
import org.firstinspires.ftc.teamcode.Utilities.Math.GeneralMatrix;
import org.firstinspires.ftc.teamcode.Utilities.Math.Matrix;
import org.firstinspires.ftc.teamcode.Utilities.Math.Vector;
import org.opencv.core.Mat;

@TeleOp(name = "MPC Testing")
public class MPCTest extends BaseOpMode {

    @Config
    public static class TestMPCParams {

        public static int N = 40;
        public static int Horizon = 5;
        public static double threshold = 0.0001;

        public static double TX = 0;
        public static double TY = 10;
        public static double TH = 0;
        public static double TVX = 0;
        public static double TVY = 0;
        public static double THV = 0;

    }

    private MPCPath test;
    private Movement drive;

    @Override
    public void externalInit() {

        DriveModel.reInit();

        drive = new FixedDriveTrain(new Vector(0, 0, 0));
        test = new MPCPath();
        test.setAccuracy(TestMPCParams.threshold);
        test.setMoveTime(TestMPCParams.Horizon);
        test.setResolution(((double) TestMPCParams.N)/TestMPCParams.Horizon);
        test.setParams(DriveConfig.DriveWheels.defaultParams);
        test.setTarget(TestMPCParams.TX, TestMPCParams.TY, TestMPCParams.TH, TestMPCParams.TVX, TestMPCParams.TVY, TestMPCParams.THV);
        test.setStart(0, 0, 0, 0, 0, 0);
        try {
            test.load("Test Path.json");
        } catch (RuntimeException e) {
            test.compile(3000);
            test.save("Test Path.json");
        }

    }

    @Override
    public void externalInitLoop() {
        test.start();
        Vector correction = test.getCorrection();

        BaseOpMode.addData("Correction X", correction.get(0));
        BaseOpMode.addData("Correction Y", correction.get(1));
        BaseOpMode.addData("Correction H", correction.get(2));

    }

    @Override
    public void externalLoop() {


        Vector correction = test.getCorrection();

        BaseOpMode.addData("Correction X", correction.get(0));
        BaseOpMode.addData("Correction Y", correction.get(1));
        BaseOpMode.addData("Correction H", correction.get(2));


        if (gamepad1.square) {
            drive.move(correction);
        }
        else {
            drive.moveRaw(new Vector(0,0,0,0));
        }

    }
}

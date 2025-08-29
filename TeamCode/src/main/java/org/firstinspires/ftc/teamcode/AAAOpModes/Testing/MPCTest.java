package org.firstinspires.ftc.teamcode.AAAOpModes.Testing;

import static org.firstinspires.ftc.teamcode.AAAOpModes.Testing.MPCTest.MPCParams.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.Motion.Controllers.ConstantSignal;
import org.firstinspires.ftc.teamcode.Motion.Controllers.MPC;
import org.firstinspires.ftc.teamcode.Motion.Controllers.Signal;
import org.firstinspires.ftc.teamcode.Motion.DriveModel;
import org.firstinspires.ftc.teamcode.Motion.Drivetrains.FixedDriveTrain;
import org.firstinspires.ftc.teamcode.Motion.Localization.Location;
import org.firstinspires.ftc.teamcode.Motion.Movement;
import org.firstinspires.ftc.teamcode.Utilities.Math.GeneralMatrix;
import org.firstinspires.ftc.teamcode.Utilities.Math.Matrix;
import org.firstinspires.ftc.teamcode.Utilities.Math.Vector;
import org.opencv.core.Mat;

@TeleOp(name = "MPC Testing")
public class MPCTest extends BaseOpMode {

    @Config
    public static class MPCParams {
        public static double QX = 100;
        public static double QY = 100;
        public static double QH = 100;
        public static double QHV = 10;
        public static double QXV = 10;
        public static double QYV = 10;

        public static double QFX = 20;
        public static double QFY = 20;
        public static double QFH = 20;
        public static double QFHV = 10;
        public static double QFXV = 10;
        public static double QFYV = 10;

        public static double R = 50;

        public static int N = 40;
        public static int Horizon = 5;
        public static double threshold = 0.0001;
        public static double lr = 1.01;
        public static double max_lambda = 1000;
        public static double max_vxx = 1000000;

        public static double TX = 0;
        public static double TY = 10;
        public static double TH = 0;
        public static double TXV = 0;
        public static double TYV = 0;
        public static double THV = 0;

    }

    private Matrix Q = new GeneralMatrix(6, 6, new double[] {
            QX, 0, 0, 0, 0, 0,
            0, QY, 0, 0, 0, 0,
            0, 0, QH, 0, 0, 0,
            0, 0, 0, QXV, 0, 0,
            0, 0, 0, 0, QYV, 0,
            0, 0, 0, 0, 0, QHV,
    });

    private Matrix QF = new GeneralMatrix(6, 6, new double[] {
            QFX, 0, 0, 0, 0, 0,
            0, QFY, 0, 0, 0, 0,
            0, 0, QFH, 0, 0, 0,
            0, 0, 0, QFXV, 0, 0,
            0, 0, 0, 0, QFYV, 0,
            0, 0, 0, 0, 0, QFHV,
    });

    private Matrix RM = Matrix.identityMatrix(3).multiplied(R);

    private MPC test;
    private Movement drive;

    @Override
    public void externalInit() {

        DriveModel.reInit();

        drive = new FixedDriveTrain(new Vector(0, 0, 0));
        test = new MPC(new ConstantSignal(new Vector(new double[] {TX, TY, TH, TXV, TYV, THV})), drive.loc, new Vector(new double[6]), Q, RM, QF, N, Horizon, threshold, lr, max_lambda, max_vxx);
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

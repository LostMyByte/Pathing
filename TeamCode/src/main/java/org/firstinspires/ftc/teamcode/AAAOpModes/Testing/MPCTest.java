package org.firstinspires.ftc.teamcode.AAAOpModes.Testing;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.Motion.Controllers.ConstantSignal;
import org.firstinspires.ftc.teamcode.Motion.Controllers.MPC;
import org.firstinspires.ftc.teamcode.Motion.Drivetrains.FixedDriveTrain;
import org.firstinspires.ftc.teamcode.Motion.Localization.Location;
import org.firstinspires.ftc.teamcode.Motion.Movement;
import org.firstinspires.ftc.teamcode.Utilities.Math.GeneralMatrix;
import org.firstinspires.ftc.teamcode.Utilities.Math.Matrix;
import org.firstinspires.ftc.teamcode.Utilities.Math.Vector;
import org.opencv.core.Mat;

@Config
public class MPCTest extends BaseOpMode {

    public static double QX = 10;
    public static double QY = 10;
    public static double QH = 10;
    public static double QHV = 0;
    public static double QXV = 0;
    public static double QYV = 0;

    public static double QFX = 20;
    public static double QFY = 20;
    public static double QFH = 20;
    public static double QFHV = 10;
    public static double QFXV = 10;
    public static double QFYV = 10;

    public static double R = 0.1;

    public static int N = 10;
    public static double threshold = 0.5;

    public static double TX = 0;
    public static double TY = 0;
    public static double TH = 0;
    public static double TXV = 0 ;
    public static double TYV = 0;
    public static double THV = 0;

    Matrix Q = new GeneralMatrix(6, 6, new double[] {
            QX, 0, 0, 0, 0, 0,
            0, QY, 0, 0, 0, 0,
            0, 0, QH, 0, 0, 0,
            0, 0, 0, QXV, 0, 0,
            0, 0, 0, 0, QYV, 0,
            0, 0, 0, 0, 0, QHV,
    });

    Matrix QF = new GeneralMatrix(6, 6, new double[] {
            QFX, 0, 0, 0, 0, 0,
            0, QFY, 0, 0, 0, 0,
            0, 0, QFH, 0, 0, 0,
            0, 0, 0, QFXV, 0, 0,
            0, 0, 0, 0, QFYV, 0,
            0, 0, 0, 0, 0, QFHV,
    });

    Matrix RM = Matrix.identityMatrix(4).multiplied(R);

    MPC test;
    Location loc;
    Movement drive;

    @Override
    public void externalInit() {
        loc = new Location(0, 0, 0);
        drive = new FixedDriveTrain(new Vector(0, 0, 0));
        test = new MPC(new ConstantSignal(new Vector(new double[] {TX, TY, TH, TXV, TYV, THV})), loc, new Vector(new double[6]), Q, RM, QF, N, threshold);
    }

    @Override
    public void externalLoop() {

        Vector correction = test.getCorrection();



    }
}

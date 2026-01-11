// Primary Author: Kieran Mattingly
package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Controllers.MPCPath;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains.FixedDriveTrain;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains.SystemModels.MechanumDrive;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains.SystemModels.SystemModel;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains.TankDriveTrain;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains.SystemModels.TankDrive;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Localization.Location;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Paths.BezierPath;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Paths.Path;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Paths.PathBuilder;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Paths.PointObstacle;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Signals.ConstantSignal;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Signals.ReferenceSignal;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Signals.SequentialSignal;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.DriveWheels;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.GeneralMatrix;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Matrix;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;

import java.io.FileNotFoundException;

@TeleOp(name = "MPC Testing")
public class MPCTest extends BaseOpMode {

    @Config
    public static class TestMPCParams {

        public static int N = 50;
        public static int Horizon = 5;
        public static int Swap = 3;
        public static double threshold = 0.0001;

        public static double TX = 0;
        public static double TY = 100;
        public static double TH = 0;
        public static double TV = 0;
        public static double THV = 0;

        public static boolean feedBack = true;

        public static boolean enabled = true;

        /*public static double OX = 0;
        public static double OY = 50;
        public static double OSize = 20;
        public static double OStrength = 500;*/

    }

    private MPCPath test;
    private FixedDriveTrain drive;

    SystemModel drivemodel;

    ReferenceSignal path;
    Location loc;


    @Override
    public void externalInit() {



        Location.llAlpha = 0; // Disable Limelight
        path = new SequentialSignal(new ConstantSignal(new Vector(0, 100, 0, TestMPCParams.TV, 0, TestMPCParams.THV)), new ConstantSignal(new Vector(TestMPCParams.TX, TestMPCParams.TY, TestMPCParams.TH, 0, 0, 0)), TestMPCParams.Swap);
        drivemodel = new MechanumDrive();
        //path = new PathBuilder(drivemodel);
        //path.addPath(new BezierPath(new Vector(0,0,0), new Vector(100, 100, Math.PI/2), new Vector(0, 0, 0), new Vector(0, 0, 0), 1.0/TestMPCParams.Horizon), TestMPCParams.Horizon);
        test = new MPCPath();
        test.setAccuracy(TestMPCParams.threshold);
        test.setMoveTime(TestMPCParams.Horizon);
        test.setResolution(((double) TestMPCParams.N)/TestMPCParams.Horizon);
        test.setParams(DriveWheels.defaultParams);
        test.setModel(drivemodel);
        test.setPath(path);
        //test.setTarget(TestMPCParams.TX, TestMPCParams.TY, TestMPCParams.TH, 0, TestMPCParams.TV, TestMPCParams.THV);
        test.setName("Test Path");
        test.setStart(0, 0, 0, 0, 0, 0);
        test.build();
        test.doTelemetry = true;
        //test.addObstacle(new PointObstacle(TestMPCParams.OX, TestMPCParams.OY, TestMPCParams.OSize, TestMPCParams.OStrength));
        try {
            test.load();

        } catch (FileNotFoundException e) {
            test.compile(500);

        } catch (RuntimeException e) {
            test.compile(500);

        }


        loc = new Location(0, 0, 0);
        loc.doTelemetry = true;
        test.setSensor(loc);
        drive = new FixedDriveTrain(loc);

    }

    @Override
    public void externalInitLoop() {
        drivemodel = new MechanumDrive();
        test.setModel(drivemodel);
        test.start();
        Vector correction = test.getCorrection(drivemodel.toStateSpace(loc));

        BaseOpMode.addData("Correction L", correction.get(0));
        BaseOpMode.addData("Correction R", correction.get(1));



    }

    @Override
    public void externalLoop() {

        Vector correction;

        Vector state = drivemodel.toStateSpace(loc);


        if (TestMPCParams.feedBack) {
            correction = test.getCorrection(state);
        }
        else {
            correction = test.getFeedForward();
        }
        BaseOpMode.addData("Correction X", correction.get(0));
        BaseOpMode.addData("Correction Y", correction.get(1));
        BaseOpMode.addData("Correction H", correction.get(2));


        if (gamepad1.square || TestMPCParams.enabled) {
            //if (drive.correctionSignal == null) drive.followController(test);
            drive.moveRaw(MechanumDrive.W.multiplied(correction));
        }
        else {
            drive.moveRaw(new Vector(0,0, 0,0));
        }

        if (gamepad1.square) {
            externalInit();
        }

    }


}

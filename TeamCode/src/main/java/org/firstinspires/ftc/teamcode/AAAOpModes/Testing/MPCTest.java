package org.firstinspires.ftc.teamcode.AAAOpModes.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.Motion.Controllers.MPCPath;
import org.firstinspires.ftc.teamcode.Motion.Drivetrains.TankDriveTrain;
import org.firstinspires.ftc.teamcode.Motion.SystemModels.MechanumDrive;
import org.firstinspires.ftc.teamcode.Motion.Movement;
import org.firstinspires.ftc.teamcode.Motion.SystemModels.TankDrive;
import org.firstinspires.ftc.teamcode.Utilities.Configuration.DriveConfig;
import org.firstinspires.ftc.teamcode.Utilities.Math.Vector;

import java.io.FileNotFoundException;

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
        public static double TV = 0;
        public static double THV = 0;

        public static boolean feedBack = false;

        public static boolean enabled = false;
    }

    private MPCPath test;
    private TankDriveTrain drive;

    @Override
    public void externalInit() {

        TankDrive.reInit();


        drive = new FixedDriveTrain(new Vector(0, 0, 0));
        test = new MPCPath();
        test.setAccuracy(TestMPCParams.threshold);
        test.setMoveTime(TestMPCParams.Horizon);
        test.setResolution(((double) TestMPCParams.N)/TestMPCParams.Horizon);
        test.setParams(DriveConfig.DriveWheels.defaultParams);
        test.setModel(new TankDrive());
        test.setTarget(TestMPCParams.TX, TestMPCParams.TY, TestMPCParams.TH, TestMPCParams.TV, TestMPCParams.THV);
        test.setStart(0, 0, 0, 0, 0);
        test.build();
        try {
            test.load("Test Path.json");

        } catch (FileNotFoundException e) {
            test.compile(500);
            test.save("Test Path.json");
        } catch (RuntimeException e) {
            test.compile(500);
            test.save("Test Path.json");
        }


        drive = new TankDriveTrain(new Vector(new double[5]));

    }

    @Override
    public void externalInitLoop() {
        TankDrive.reInit();
        test.start();
        Vector correction = test.getCorrection(drive.loc.getPositionForTankDrive());

        BaseOpMode.addData("Correction L", correction.get(0));
        BaseOpMode.addData("Correction R", correction.get(1));


    }

    @Override
    public void externalLoop() {

        Vector correction;
        if (TestMPCParams.feedBack) {
            correction = test.getCorrection(drive.loc.getPositionForTankDrive());
        }
        else {
            correction = test.getFeedForward();
        }
        BaseOpMode.addData("Correction L", correction.get(0));
        BaseOpMode.addData("Correction R", correction.get(1));



        if (gamepad1.square || TestMPCParams.enabled) {
            drive.moveRaw(correction);
        }
        else {
            drive.move(new Vector(0,0));
        }

    }
}

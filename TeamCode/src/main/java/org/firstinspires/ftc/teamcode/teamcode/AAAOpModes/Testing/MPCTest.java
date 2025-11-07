// Primary Author: Kieran Mattingly
package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Controllers.MPCPath;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains.TankDriveTrain;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains.SystemModels.TankDrive;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Localization.Location;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.DriveWheels;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;

import java.io.FileNotFoundException;

@TeleOp(name = "MPC Testing")
public class MPCTest extends BaseOpMode {

    @Config
    public static class TestMPCParams {

        public static int N = 40;
        public static int Horizon = 5;
        public static double threshold = 0.0001;

        public static double TX = 0;
        public static double TY = -10;
        public static double TH = Math.PI;
        public static double TV = 0;
        public static double THV = 0;

        public static boolean feedBack = false;

        public static boolean enabled = false;
    }

    private MPCPath test;
    private TankDriveTrain drive;

    TankDrive drivemodel;

    Location loc;

    @Override
    public void externalInit() {

        drivemodel = new TankDrive();
        test = new MPCPath();
        test.setAccuracy(TestMPCParams.threshold);
        test.setMoveTime(TestMPCParams.Horizon);
        test.setResolution(((double) TestMPCParams.N)/TestMPCParams.Horizon);
        test.setParams(DriveWheels.defaultParams);
        test.setModel(drivemodel);
        test.setTarget(TestMPCParams.TX, TestMPCParams.TY, TestMPCParams.TH, TestMPCParams.TV, TestMPCParams.THV);
        test.setName("Test Path");
        test.setStart(0, 0, Math.PI, 0, 0);
        test.build();
        try {
            test.load();

        } catch (FileNotFoundException e) {
            test.compile(500);

        } catch (RuntimeException e) {
            test.compile(500);

        }


        loc = new Location(0, 0, Math.PI);
        test.setSensor(loc);
        drive = new TankDriveTrain();

    }

    @Override
    public void externalInitLoop() {
        drivemodel = new TankDrive();
        test.setModel(drivemodel);
        test.start();
        Vector correction = test.getCorrection(loc.getPositionForTankDrive());

        BaseOpMode.addData("Correction L", correction.get(0));
        BaseOpMode.addData("Correction R", correction.get(1));


    }

    @Override
    public void externalLoop() {

        Vector correction;

        if (TestMPCParams.feedBack) {
            correction = test.getCorrection(loc.getPositionForTankDrive());
        }
        else {
            correction = test.getFeedForward();
        }
        BaseOpMode.addData("Correction L", correction.get(0));
        BaseOpMode.addData("Correction R", correction.get(1));

        if (gamepad1.square || TestMPCParams.enabled) {
            if (drive.correctionSignal == null) drive.followController(test);
        }
        else {
            drive.move(new Vector(0,0));
        }

    }


}

// Primary Author: Kieran Mattingly
package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Controllers.MPCPath;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains.TankDriveTrain;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains.SystemModels.SystemModel;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains.SystemModels.TankDrive;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Localization.Location;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.DriveWheels;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;

import java.io.FileNotFoundException;


@Autonomous(name = "Blue Front Auto")
public class BlueFrontAuto extends BaseOpMode {

    @Config
    public static class BlueFrontPositions {
        public static double X0 = 0;
        public static double Y0 = 0;
        public static double H0 = 135;


        public static double X1 = 00;
        public static double Y1 = 05;
        public static double H1 = 90;

        public static double X2 = 0;
        public static double Y2 = 0;
        public static double H2 = 90;

        public static double XLaunch = 0;
        public static double YLaunch = 0;
        public static double HLaunch = 180;

        public static double XLeave = 0;
        public static double YLeave = 0;
        public static double HLeave = 180;

        public static double V = 0;

        public static double Tlaunch = 5;
        public static double T1 = 5;
        public static double T1launch = 5;
        public static double T2 = 5;
        public static double T2launch = 5;
        public static double Tleave= 5;

        public static double accuracy = 0.001;

        public static double resolution = 10;

        public static boolean alwaysCompile = false;
    }

    enum States {
        Launch,
        Spike1,
        Spike1Launch,
        Spike2,
        Spike2Launch,
        Leave
    }

    ElapsedTime stateTime;

    MPCPath launch;
    MPCPath spike1;
    MPCPath spike1Launch;
    MPCPath spike2;
    MPCPath spike2Launch;

    MPCPath leave;

    MPCPath[] allPaths;

    SystemModel driveModel = new TankDrive();
    States state;

    TankDriveTrain drive;

    Location loc;

    @Override
    public void externalInit() {
        stateTime = new ElapsedTime();

        loc = new Location(0, 0, 0);
        drive = new TankDriveTrain();

        initPaths();

        state = States.Launch;

        if (BlueFrontPositions.alwaysCompile) {
            compilePaths();
        }
        else {
            loadPaths();
        }


    }

    public void initPaths() {
        launch = new MPCPath();
        launch.setMoveTime(BlueFrontPositions.Tlaunch);
        launch.setStart(BlueFrontPositions.X0, BlueFrontPositions.Y0, Math.toRadians(BlueFrontPositions.H0), 0, 0);
        launch.setTarget(BlueFrontPositions.XLaunch, BlueFrontPositions.YLaunch, Math.toRadians(BlueFrontPositions.HLaunch), 0, 0);
        launch.setName("BFLaunch");

        spike1 = new MPCPath();
        spike1.setMoveTime(BlueFrontPositions.T1);
        spike1.continueFrom(launch);
        spike1.setTarget(BlueFrontPositions.X1, BlueFrontPositions.Y1, Math.toRadians(BlueFrontPositions.H1), BlueFrontPositions.V, 0);
        spike1.setName("BFSpike1");

        spike1Launch = new MPCPath();
        spike1Launch.setMoveTime(BlueFrontPositions.T1launch);
        spike1Launch.continueFrom(spike1);
        spike1Launch.setTarget(BlueFrontPositions.XLaunch, BlueFrontPositions.YLaunch, Math.toRadians(BlueFrontPositions.HLaunch), 0, 0);
        spike1Launch.setName("BFSpike1Launch");

        spike2 = new MPCPath();
        spike2.setMoveTime(BlueFrontPositions.T2);
        spike2.continueFrom(spike1Launch);
        spike2.setTarget(BlueFrontPositions.X2, BlueFrontPositions.Y2, Math.toRadians(BlueFrontPositions.H2), BlueFrontPositions.V, 0);
        spike2.setName("BFSpike2");

        spike2Launch = new MPCPath();
        spike2Launch.setMoveTime(BlueFrontPositions.T2launch);
        spike2Launch.continueFrom(spike2);
        spike2Launch.setTarget(BlueFrontPositions.XLaunch, BlueFrontPositions.YLaunch, Math.toRadians(BlueFrontPositions.HLaunch), 0, 0);
        spike2Launch.setName("BFSpike2Launch");

        leave = new MPCPath();
        leave.setMoveTime(BlueFrontPositions.Tleave);
        leave.continueFrom(spike2Launch);
        leave.setTarget(BlueFrontPositions.XLeave, BlueFrontPositions.YLeave, Math.toRadians(BlueFrontPositions.HLeave), 0, 0);
        leave.setName("BFLeave");

        allPaths = new MPCPath[]{launch, spike1, spike1Launch, spike2, spike2Launch, leave};

        for (MPCPath path : allPaths) {
            path.setAccuracy(BlueFrontPositions.accuracy);
            path.setResolution(BlueFrontPositions.resolution);
            path.setStopTime(1);
            path.setParams(DriveWheels.defaultParams);
            path.setModel(driveModel);
            path.setSensor(loc);
        }
    }

    Vector position;

    public void compilePaths() {
        for (MPCPath path : allPaths) {
            path.build();
            path.compile(500);
        }
    }
    public void loadPaths() {
        for (MPCPath path : allPaths) {
            try {
                path.build();
                path.load();
            }
            catch (FileNotFoundException e) {
                path.compile(500);
                path.save();
            }
        }
    }
    public void savePaths() {
        for (MPCPath path : allPaths) {
            path.save();
        }
    }

    @Override
    public void externalInitLoop() {
        position = loc.getPositionForTankDrive();
        stateTime.reset();

        launch.start();
        launch.getCorrection(position);

    }

    @Override
    public void externalLoop() {
        BaseOpMode.addData("State", state);
        position = loc.getPositionForTankDrive();
        stateMachine();
    }

    public void stateMachine() {
        switch (state) {
            case Launch: launch(); break;
            case Spike1: spike1(); break;
            case Spike1Launch: spike1Launch(); break;
            case Spike2: spike2(); break;
            case Spike2Launch: spike2Launch(); break;
            case Leave: leave(); break;
        }
    }

    public void setState(States state) {
        this.stateTime.reset();
        this.state = state;
    }


    public void launch() {
        drive.followController(launch);
        if (launch.getState() == MPCPath.ControllerStates.Finished) setState(States.Spike1);
    }
    private void spike1() {
        drive.followController(spike1);
        if (spike1.getState() == MPCPath.ControllerStates.Finished) setState(States.Spike1Launch);
    }
    private void spike1Launch() {
        drive.followController(spike1Launch);
        if (spike1Launch.getState() == MPCPath.ControllerStates.Finished) setState(States.Spike2);
    }
    private void spike2() {
        drive.followController(spike2);
        if (spike2.getState() == MPCPath.ControllerStates.Finished) setState(States.Spike2Launch);
    }
    private void spike2Launch() {
        drive.followController(spike2Launch);
        if (spike2Launch.getState() == MPCPath.ControllerStates.Finished) setState(States.Leave);
    }
    private void leave() {
        drive.followController(leave);
    }

}

package org.firstinspires.ftc.teamcode.AAAOpModes.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.Motion.Controllers.MPCPath;
import org.firstinspires.ftc.teamcode.Motion.Drivetrains.TankDriveTrain;
import org.firstinspires.ftc.teamcode.Motion.Movement;
import org.firstinspires.ftc.teamcode.Motion.SystemModels.SystemModel;
import org.firstinspires.ftc.teamcode.Motion.SystemModels.TankDrive;
import org.firstinspires.ftc.teamcode.Utilities.Configuration.DriveConfig;
import org.firstinspires.ftc.teamcode.Utilities.Math.Vector;

import java.io.FileNotFoundException;


@Autonomous(name = "Red Front Auto")
public class RedFrontAuto extends BaseOpMode {

    @Config
    public static class RedPositions {
        public static double X0 = 0;
        public static double Y0 = 0;
        public static double H0 = 0;


        public static double X1 = -60;
        public static double Y1 = 120;
        public static double H1 = 3* Math.PI/4;

        public static double X2 = -100;
        public static double Y2 = 130;
        public static double H2 = 3* Math.PI/4;

        public static double XLaunch = -15;
        public static double YLaunch = 120;
        public static double HLaunch = Math.PI/4;

        public static double XLeave = 70;
        public static double YLeave = 50;
        public static double HLeave = Math.PI/4;

        public static double V = 0;

        public static double Tlaunch = 5;
        public static double T1 = 3;
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

    @Override
    public void externalInit() {
        stateTime = new ElapsedTime();

        drive = new TankDriveTrain(new Vector(new double[] {RedPositions.X0, RedPositions.Y0, RedPositions.H0, 0, 0}));

        initPaths();

        state = States.Launch;

        loadPaths();

        if (RedPositions.alwaysCompile) {
            compilePaths();
        }


    }

    public void initPaths() {
        launch = new MPCPath();
        launch.setMoveTime(RedPositions.Tlaunch);
        launch.setStart(RedPositions.X0, RedPositions.Y0, RedPositions.H0, 0, 0);
        launch.setTarget(RedPositions.XLaunch, RedPositions.YLaunch, RedPositions.HLaunch, 0, 0);
        launch.setName("Launch");

        spike1 = new MPCPath();
        spike1.setMoveTime(RedPositions.T1);
        spike1.continueFrom(launch);
        spike1.setTarget(RedPositions.X1, RedPositions.Y1, RedPositions.H1, RedPositions.V, 0);
        spike1.setName("Spike1");

        spike1Launch = new MPCPath();
        spike1Launch.setMoveTime(RedPositions.T1launch);
        spike1Launch.continueFrom(spike1);
        spike1Launch.setTarget(RedPositions.XLaunch, RedPositions.YLaunch, RedPositions.HLaunch, 0, 0);
        spike1Launch.setName("Spike1Launch");

        spike2 = new MPCPath();
        spike2.setMoveTime(RedPositions.T2);
        spike2.continueFrom(spike1Launch);
        spike2.setTarget(RedPositions.X2, RedPositions.Y2, RedPositions.H2, RedPositions.V, 0);
        spike2.setName("Spike2");

        spike2Launch = new MPCPath();
        spike2Launch.setMoveTime(RedPositions.T2launch);
        spike2Launch.continueFrom(spike2);
        spike2Launch.setTarget(RedPositions.XLaunch, RedPositions.YLaunch, RedPositions.HLaunch, 0, 0);
        spike2Launch.setName("Spike2Launch");

        leave = new MPCPath();
        leave.setMoveTime(RedPositions.Tleave);
        leave.continueFrom(spike2Launch);
        leave.setTarget(RedPositions.XLeave, RedPositions.YLeave, RedPositions.HLeave, 0, 0);
        leave.setName("Leave");

        allPaths = new MPCPath[]{launch, spike1, spike1Launch, spike2, spike2Launch, leave};

        for (MPCPath path : allPaths) {
            path.setAccuracy(RedPositions.accuracy);
            path.setResolution(RedPositions.resolution);
            path.setParams(DriveConfig.DriveWheels.defaultParams);
            path.setModel(driveModel);
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
        position = drive.loc.getPositionForTankDrive();
        stateTime.reset();

        launch.start();
        launch.getCorrection(position);

    }

    @Override
    public void externalLoop() {
        BaseOpMode.addData("State", state);
        position = drive.loc.getPositionForTankDrive();
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
        drive.moveRaw(launch.getCorrection(position));
        if (launch.getState() == MPCPath.ControllerStates.Finished) setState(States.Spike1);
    }
    private void spike1() {
        drive.moveRaw(spike1.getCorrection(position));
        if (spike1.getState() == MPCPath.ControllerStates.Finished) setState(States.Spike1Launch);
    }
    private void spike1Launch() {
        drive.moveRaw(spike1Launch.getCorrection(position));
        if (spike1Launch.getState() == MPCPath.ControllerStates.Finished) setState(States.Spike2);
    }
    private void spike2() {
        drive.moveRaw(spike2.getCorrection(position));
        if (spike2.getState() == MPCPath.ControllerStates.Finished) setState(States.Spike2Launch);
    }
    private void spike2Launch() {
        drive.moveRaw(spike2Launch.getCorrection(position));
        if (spike2Launch.getState() == MPCPath.ControllerStates.Finished) setState(States.Leave);
    }
    private void leave() {
        drive.moveRaw(leave.getCorrection(position));
    }

}

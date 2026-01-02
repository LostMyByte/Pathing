// Primary Author: Kieran Mattingly
package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous;

import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.RedBackAuto.RedPositions.H0;
import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.RedBackAuto.RedPositions.X0;
import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.RedBackAuto.RedPositions.X1;
import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.RedBackAuto.RedPositions.Y0;
import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.RedBackAuto.RedPositions.Y1;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Controllers.MPCPath;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains.TankDriveTrain;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains.SystemModels.SystemModel;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains.SystemModels.TankDrive;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Localization.Location;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.IntakeMagazine;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Constants;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.DriveWheels;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;

import java.io.FileNotFoundException;


@Autonomous(name = "NormalRedBackAuto")
public class RedBackAuto extends BaseOpMode {

    @Config
    public static class RedPositions {
        public static double X0 = 161;
        public static double Y0 = 341;
        public static double H0 = 0;


        public static double X1 = 150;
        public static double Y1 = 170;
        public static double H1 = Math.toRadians(40);

        public static double X2 = -120;
        public static double Y2 = 100;
        public static double H2 = 3* Math.PI/4;

        public static double XLaunch = 0;
        public static double YLaunch = 70;
        public static double HLaunch = Math.PI/4;

        public static double XLeave = 30;
        public static double YLeave = 30;
        public static double HLeave = Math.PI/4;

        public static double V = 0;

        public static double Tlaunch = 4;
        public static double T1 = 3;
        public static double T1launch = 3;
        public static double T2 = 5;
        public static double T2launch = 5;
        public static double Tleave= 5;

        public static double accuracy = 0.0001;

        public static double resolution = 10;

        public static boolean alwaysCompile = false;
        public static double speedscale = 0.1;
    }

    enum States {
        Launch,
        Spike1Intermittent,
        Spike1,
        Spike1Launch,
        Spike2,
        Spike2Launch,
        Leave
    }

    ElapsedTime stateTime;

    MPCPath launch;
    MPCPath spike1;
    MPCPath spike1I;
    MPCPath spike1Launch;
    MPCPath spike2;
    MPCPath spike2Launch;

    MPCPath leave;

    MPCPath[] allPaths;

    SystemModel driveModel = new TankDrive();
    States state;

    TankDriveTrain drive;

    Location loc;

    MPCPath.MPCParams slow;
    Shooter shooter;
    IntakeMagazine intake;
    boolean firstLoop1 = true;

    @Override
    public void externalInit() {
        Constants.team = Constants.Team.RED;
        stateTime = new ElapsedTime();

        loc = new Location(143, 341, 0);
        loc.doTelemetry = true;
        drive = new TankDriveTrain();
        slow = DriveWheels.defaultParams.copy();
        slow.scaleVelocity(RedPositions.speedscale);
        initPaths();
        shooter = new Shooter(hardwareMap, Constants.Team.RED);
        shooter.setState(Shooter.ShooterStates.NOTACTIVE);
        intake = new IntakeMagazine(hardwareMap);
        intake.setState(IntakeMagazine.IntakeMagazineStates.DONOTHING);



        state = States.Launch;

        compilePaths();
        //loadPaths();
        loc.setPosition(X0, Y0, H0);

        if (RedPositions.alwaysCompile) {
            compilePaths();
        }


    }

    public void initPaths() {
        launch = new MPCPath();
        launch.setMoveTime(RedPositions.Tlaunch);
        launch.setStart(X0, Y0, H0, 0, 0);
        launch.setTarget(X1, Y1, RedPositions.H1, 0, 0);
        launch.setName("Launch");

        spike1I = new MPCPath();
        spike1I.setMoveTime(RedPositions.T1);
        spike1I.continueFrom(launch);
        spike1I.setTarget(95, 268, Math.PI/2, RedPositions.V, 0);
        spike1I.setName("Spike1");

        spike1 = new MPCPath();
        spike1.setMoveTime(RedPositions.T1);
        spike1.setStart(155, 170, 0.2145,0,0);
        spike1.setTarget(X1, Y1, RedPositions.H1, RedPositions.V, 0);
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

        allPaths = new MPCPath[]{launch, spike1, spike1I, spike1Launch, spike2, spike2Launch, leave};

        for (MPCPath path : allPaths) {
            path.setAccuracy(RedPositions.accuracy);
            path.setResolution(RedPositions.resolution);
            path.setStopTime(1);
            path.setParams(DriveWheels.defaultParams);
            path.setModel(driveModel);
        }

        spike1.setParams(slow);
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
        launch.doTelemetry = true;
        //spike1I.doTelemetry = true;
        launch.getCorrection(position);

    }


    @Override
    public void externalLoop() {
        BaseOpMode.addData("State", state);
        position = loc.getPositionForTankDrive();
        stateMachine();
    }

    public void externalStart(){
        intake.setState(IntakeMagazine.IntakeMagazineStates.IDLE);
    }

    @Override
    public void externalStop(){
        blackboard.put("AutoEndPos", loc.getPosition());
        blackboard.put("Team", Constants.Team.RED);
    }

    public void stateMachine() {
        switch (state) {
            case Launch: launch(); break;
            case Spike1Intermittent: spike1I(); break;
            case Spike1: spike1(); break;
            case Spike1Launch: spike1Launch(); break;
            case Spike2: spike2(); break;
            case Spike2Launch: spike2Launch(); break;
            case Leave: leave(); break;
        }
        shooter.recieveOdoInputs(loc.getPosX(), loc.getPosY(),loc.getPosH(), new Vector(0,0,0), loc.getVelH());
    }

    public void setState(States state) {
        this.stateTime.reset();
        this.state = state;
        firstLoop1 = true;
    }


    public void launch() {
        drive.moveRaw(launch.getCorrection(position));
        //shooter.setState(Shooter.ShooterStates.ACTIVE);
        if(intake.getNumBalls() == 0){
            setState(States.Spike1Intermittent);
        } else if (launch.getState() == MPCPath.ControllerStates.Finished && intake.getState() == IntakeMagazine.IntakeMagazineStates.IDLE){
            //intake.setState(IntakeMagazine.IntakeMagazineStates.SHOOTING);
            firstLoop1 = false;
        }
    }
    private void spike1I() {
        drive.moveRaw(spike1I.getCorrection(position));
        if (spike1.getState() == MPCPath.ControllerStates.Finished) {
            //setState(States.Spike1);
        }
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


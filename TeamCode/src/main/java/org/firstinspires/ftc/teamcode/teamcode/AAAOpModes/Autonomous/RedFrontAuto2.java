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
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.TeliOpDrivetrain;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.DriveWheels;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;

import java.io.FileNotFoundException;


@Autonomous(name = "Red Front Auto 2")
public class RedFrontAuto2 extends BaseOpMode {


    enum States {
        Launch,
        Spike1,
        Spike1Launch,
        Spike2,
        Spike2Launch,
        Leave
    }
    ElapsedTime stateTime;
    States state = States.Launch;
    TeliOpDrivetrain drive;
    Location loc;
    boolean firstLoop1 = true;
    boolean firstLoop2 = true;

    @Override
    public void externalInit() {
        stateTime = new ElapsedTime();

        loc = new Location(0,0,0);
        drive = new TeliOpDrivetrain(hardwareMap, 0);

        state = States.Launch;



    }



    @Override
    public void externalLoop() {
        BaseOpMode.addData("State", state);
        drive.updateOdo(loc.getPosX(), loc.getPosY(), loc.getPosH());
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
        firstLoop1 = true;
        firstLoop2 = true;
    }


    public void launch() {
        if (stateTime.seconds() < 3){
            drive.driveDumb(-155,0,.6,firstLoop1);
            firstLoop1 = false;
        } else {
            drive.driveDumb(0,Math.toRadians(-20), .4, firstLoop2);
            firstLoop2 = false;
        }
    }
    private void spike1() {
    }
    private void spike1Launch() {
    }
    private void spike2() {
    }
    private void spike2Launch() {
    }
    private void leave() {

    }

}

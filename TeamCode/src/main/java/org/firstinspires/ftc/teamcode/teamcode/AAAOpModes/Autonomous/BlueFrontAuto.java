// Primary Author: Kieran Mattingly
package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Localization.Location;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.IntakeMagazine;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.TeliOpDrivetrain;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Constants;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;


@Autonomous(name = "Blue Front Auto")
public class BlueFrontAuto extends BaseOpMode {


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
    Shooter shooter;
    IntakeMagazine intake;
    boolean firstLoop1 = true;
    boolean firstLoop2 = true;

    @Override
    public void externalInit() {
        stateTime = new ElapsedTime();

        Constants.team = Constants.Team.BLUE;
        loc = new Location(-40,78,-Math.PI/2);
        loc.doTelemetry = true;
        drive = new TeliOpDrivetrain(hardwareMap, 0);

        state = States.Spike1;

        intake = new IntakeMagazine(hardwareMap);
        shooter = new Shooter(hardwareMap, Constants.Team.BLUE);
        intake.setState(IntakeMagazine.IntakeMagazineStates.DONOTHING);

        loc.setPosition(-40,78,-Math.PI/2);

    }

    @Override
    public void externalStart(){
        stateTime.reset();
    }

    @Override
    public void externalStop(){
        blackboard.put("AutoEndPos", loc.getPosition());
        blackboard.put("Team", Constants.Team.BLUE);
    }



    @Override
    public void externalLoop() {
        BaseOpMode.addData("State", state);
        drive.updateOdo(loc.getPosX(), loc.getPosY(), loc.getPosH());
        BaseOpMode.addData("headingVelocity", loc.getVelH());
        shooter.recieveOdoInputs(loc.getPosX(), loc.getPosY(), loc.getPosH(),new Vector(0,0,0), loc.getVelH());
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


    public void launch(){
        if (stateTime.seconds() > 15){
            shooter.setState(Shooter.ShooterStates.NOTACTIVE);
        } else {
            shooter.setState(Shooter.ShooterStates.ACTIVE);
        }
        if (stateTime.seconds() > 8 && firstLoop1){
            intake.setState(IntakeMagazine.IntakeMagazineStates.SHOOTING);
            firstLoop1 = false;
        } else if (stateTime.seconds() < 8) {
            intake.setState(IntakeMagazine.IntakeMagazineStates.IDLE);
        }
    }
    private void spike1() {
        shooter.setState(Shooter.ShooterStates.NOTACTIVE);
        if (stateTime.seconds() < 1.6){
            drive.drive(0.4, 0,1);
        } else {
            drive.drive(0,0,0);
            setState(States.Launch);
        }
    }
    private void spike1Launch() {
        drive.dumbDriveToPos(135, 195, .7);
        if (stateTime.seconds() > 3){
            intake.setState(IntakeMagazine.IntakeMagazineStates.SHOOTING);
        }
        if (stateTime.seconds() > 5){
            //setState();
        }
    }
    private void spike2() {
    }
    private void spike2Launch() {
    }
    private void leave() {

    }

}

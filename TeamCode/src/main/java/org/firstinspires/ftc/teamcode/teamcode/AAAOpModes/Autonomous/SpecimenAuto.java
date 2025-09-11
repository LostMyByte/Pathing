
package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous;



import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.AAA_Paths.Path.Intake1;
import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.AAA_Paths.Path.Intake2;
import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.AAA_Paths.Path.PushSpikes;
import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.AAA_Paths.Path.Score1;
import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.AAA_Paths.Path.Score2;
import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.AAA_Paths.Path.Score3;
import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.AAA_Paths.Path.StopState;
import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.AAA_Paths.Path.Submersible;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.MainIntake.IntakeStates.EXTENDEDACTIVE;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.MainIntake.IntakeStates.EXTENDEDNOTACTIVE;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.MainIntake.IntakeStates.EXTENDEDREVERSED;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.MainIntake.IntakeStates.EXTENDEDUP;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.MainIntake.IntakeStates.HOMEUP;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.MainIntake.IntakeStates.HOMEUPIN;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.MainIntake.IntakeStates.SEARCHNOYELLOW;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Scoring.ScoringStates.HIGHRUNG;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Scoring.ScoringStates.HIGHRUNGSCORE;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Scoring.ScoringStates.HIGHRUNGSCOREBACK;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Scoring.ScoringStates.HOME;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Scoring.ScoringStates.INTAKESPECIMENFRONT;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Scoring.ScoringStates.INTAKESPECIMENFRONTREADY;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Scoring.ScoringStates.INTAKESPECIMENREADY;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Scoring.ScoringStates.STARTAUTO;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.teamcode.KCP.Localization.TwoWheelOdometry;
import org.firstinspires.ftc.teamcode.teamcode.KCP.Movement;

import org.firstinspires.ftc.teamcode.teamcode.Subsystems.MainIntake;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Scoring;

@Autonomous(name = "SpecimenAuto6")
public class SpecimenAuto extends BaseOpMode {

    Movement drive;
    Scoring scoring;
    MainIntake intake;
    AAA_Paths.Path state = Score1;
    ElapsedTime timeWaste = new ElapsedTime();
    ElapsedTime totalTime;
    boolean done1 = false;
    boolean done2 = false;
    boolean done3 = false;
    boolean firstSubCycle = true;
    int cycles = 0;


    double targetHeading = Math.PI/2;

    @Override
    public void externalInit() {
        drive = new Movement(Constants.halfRobotSizeX, 62.5,targetHeading);
        drive.setActiveCorrectionMethod(Movement.CorrectionMethods.Profiled);
        telemetry.update();
        totalTime = new ElapsedTime();
        timeWaste = new ElapsedTime();

        totalTime.reset();

        //TODO DELETE
        intake = new MainIntake(hardware);
        intake.setState(HOMEUP);
        scoring = new Scoring(hardware);
        scoring.setState(STARTAUTO);
        drive.setHeading(targetHeading);

        scoring.resetSlides();
        intake.resetSlides();



        BaseOpMode.addData("X", TwoWheelOdometry.x());
        BaseOpMode.addData("Y", TwoWheelOdometry.y());
        BaseOpMode.addData("Heading", TwoWheelOdometry.heading());

    }


    @Override
    public void externalInitLoop() {
    }

    @Override
    public void externalStart() {
        timeWaste.reset();
        totalTime.reset();
        scoring.stopResetting();
        intake.stopResetting();
    }

    @Override
    public void externalLoop() {
        stateMachine();
        BaseOpMode.addData("timeWaste", timeWaste.seconds());
        BaseOpMode.addData("State", state);
        BaseOpMode.addData("X", TwoWheelOdometry.x());
        BaseOpMode.addData("Y", TwoWheelOdometry.y());
        BaseOpMode.addData("H", TwoWheelOdometry.heading());
        BaseOpMode.addData("targetHeading", targetHeading);
        BaseOpMode.addData("intakeState", intake.getState());
        BaseOpMode.addData("scoringState", scoring.getState());

        if (totalTime.seconds() > 29){
            intake.setState(HOMEUP);
            //scoring.setState(HOME);
        }
    }

    @Override
    public void externalStop() {

    }

    public void stateMachine() {
        switch (state) {
            case Score1:
                Score1();
                break;
            case Intake2:
                intake2();
                break;
            case Score2:
                score2();
                break;
            case Score3:
                scoreFirstSpike();
                break;
            case PushSpikes:
                moveSpikes();
                break;
            case Intake1:
                intake1();
                break;
            case StopState:
                stopState();
                break;
            case Submersible:
                submersibleSpecimen();
                break;
        }
    }

    public void Score1() {
        intake.setState(EXTENDEDUP);
        intake.setSlidesLengthInches(2);
        if(!done1){
            scoring.setState(HIGHRUNG);
            done1 = true;
        }

        drive.holdPosition(45, 70,targetHeading, 1);
        //cycles++;
        if (timeWaste.seconds() > 1.2){
            scoring.setState(HOME);
            setState(Submersible);

        }else if(timeWaste.seconds()>1){
            if(!done3){
                scoring.setState(HIGHRUNGSCORE);
                done3 = true;
            }
        }

    }

    double holdY = 70;
    double holdX = 45;
    double oldOffset = 0;
    public void submersibleSpecimen() {
        Constants.team = Constants.Team.RED;

        if (intake.getState() != SEARCHNOYELLOW && !intake.visionSampleReady){
        intake.setState(SEARCHNOYELLOW);
        done1 = false;
    }
        if (drive.getVelocity()[0] < 0.1 && holdX == 45) {
        holdX = drive.getX() + 2;
    }
        if (intake.getState() == SEARCHNOYELLOW) {
        if (intake.blockMovement != oldOffset) {
            oldOffset = intake.blockMovement;
            holdY += MainIntake.CameraMount.DriveAlpha*(drive.getY() + intake.blockMovement - holdY);
        }
        BaseOpMode.addData("Holding X", holdX);
        BaseOpMode.addData("Holding Y", holdY);
        drive.holdPosition(holdX, holdY, Math.PI/2, 1);
        }
        if (intake.visionSampleReady || timeWaste.seconds() > 2){
            intake.setState(HOMEUPIN);
            setState(PushSpikes);
        }
    }

    public void pushSpikes(){
        if(timeWaste.seconds() < .5){
            drive.holdPosition(30, 70, Math.PI/2, .7);
        } else if(timeWaste.seconds() < 1.2 && timeWaste.seconds() > .5) {
            drive.holdPosition(26, 40, Math.PI/2, .75);
        } else if (timeWaste.seconds() > 1.2 && timeWaste.seconds() < 2){
            drive.holdPosition(50, 40, Math.PI/2, .75);
        } else if (timeWaste.seconds() > 2 && timeWaste.seconds() < 2.7){
            drive.holdPosition(56, 25, Math.PI/2, .75);
        } else if (timeWaste.seconds() > 2.7 && timeWaste.seconds() < 3.9){
            drive.holdPosition(20, 25, Math.PI/2, .75);
        } else if (timeWaste.seconds() > 3.9 && timeWaste.seconds() < 4.3){
            drive.holdPosition(35, 30, Math.PI/2, .75);}
        else if (timeWaste.seconds() > 4.3 && timeWaste.seconds() < 5.5){
            drive.holdPosition(51, 18, Math.PI/2, .75);
        } else if (timeWaste.seconds() > 5.5 && timeWaste.seconds() < 6.5) {
            drive.holdPosition(20, 18, Math.PI / 2, .75);
        } else if (timeWaste.seconds() > 6.5 && timeWaste.seconds() < 7.2){
            drive.holdPosition(50, 20, Math.PI/2, .75);
        } else if (timeWaste.seconds() > 7.2 && timeWaste.seconds() < 7.8){
            drive.holdPosition(59, 2, Math.PI/2, .75);
        } else if (timeWaste.seconds() > 7.8&& timeWaste.seconds() < 8.9){
            scoring.setState(INTAKESPECIMENREADY);
            drive.holdPosition(20, 2, Math.PI/2, .75);
        } else if (timeWaste.seconds() > 8.9){
            setState(Intake1);
        }
    }

    public void moveSpikes(){
        if(timeWaste.seconds() < 1.8){
            drive.holdPosition(50, 38.7, -1.2241, 1);
            if(timeWaste.seconds() > 1.5){
                intake.setState(EXTENDEDNOTACTIVE);
                intake.setSlidesLengthInches(10);
            }
        } else if(timeWaste.seconds() > 1.8 && timeWaste.seconds() < 2.9) {
            if (timeWaste.seconds() < 2){
                intake.setState(EXTENDEDREVERSED);
            } else {
                intake.setState(EXTENDEDUP);
                intake.setSlidesLengthInches(0);
            }
            drive.holdPosition(86.31, 32.54, -Math.PI/2, 1);
        } else if (timeWaste.seconds() > 2.9 && timeWaste.seconds() < 3.6){
            drive.holdPosition(87, 21, -Math.PI/2, 1);
            if (timeWaste.seconds() > 3.2){
                intake.setState(EXTENDEDACTIVE);
            }
        } else if (timeWaste.seconds() > 3.6 && timeWaste.seconds() < 4.5){
            drive.holdPosition(55, 22, -Math.PI/2, 1);
            if (timeWaste.seconds() < 4.3){
                intake.setState(EXTENDEDACTIVE);
                intake.setSlidesLengthInches(16);
            } else {
                intake.setState(EXTENDEDREVERSED);
            }

        } else if (timeWaste.seconds() > 4.5 && timeWaste.seconds() < 5.4){
            drive.holdPosition(86, 22, -Math.PI/2, 1);
            intake.setState(EXTENDEDUP);
            intake.setSlidesLengthInches(0);
        } else if (timeWaste.seconds() > 5.4 && timeWaste.seconds() < 6.1){
            drive.holdPosition(88, 12, -Math.PI/2, 1);}
        else if (timeWaste.seconds() > 6.1 && timeWaste.seconds() < 7){
            drive.holdPosition(60, 12, -Math.PI/2, 1);
            if (timeWaste.seconds() < 6.8) {
                intake.setState(EXTENDEDACTIVE);
                intake.setSlidesLengthInches(16);
            } else {
                intake.setState(EXTENDEDREVERSED);
            }

        } else if (timeWaste.seconds() > 7&& timeWaste.seconds() < 8.1) {
            if (timeWaste.seconds() < 7.8){
                intake.setState(HOMEUP);
                drive.holdPosition(88, 12, -Math.PI/2, 1);
            } else {
                drive.holdPosition(90, 7, -Math.PI/2, 1);

            }
        } else if (timeWaste.seconds() > 8.1 && timeWaste.seconds() < 9.1){
            drive.holdPosition(40, 8, -Math.PI/2, 1);
        } else if (timeWaste.seconds() > 9.1){
            scoring.setState(INTAKESPECIMENFRONTREADY);
            setState(Intake1);
        }
    }

    public void moveSpikesWorse(){
        if(timeWaste.seconds() < 1.8){
            drive.holdPosition(41.54, 60.16, -0.4767, 1);
            if(timeWaste.seconds() > 1.4){
                intake.setState(EXTENDEDNOTACTIVE);
                intake.setSlidesLengthInches(15);
            }
        } else if(timeWaste.seconds() > 1.8 && timeWaste.seconds() < 3) {
            if (timeWaste.seconds() < 1.9){
                intake.setState(EXTENDEDREVERSED);
                drive.holdPosition(41.54, 60.16, -0.4767, 1);
            } else {
                intake.setState(HOMEUP);
                drive.holdPosition(30.28, 52.2, -0.7756, .1);
            }
        } else if (timeWaste.seconds() > 3 && timeWaste.seconds() < 3.8){
            drive.holdPosition(30.28, 52.2, 0.7756, .75);
            intake.setState(EXTENDEDACTIVE);
            intake.setTargetSlidesLength(13+(timeWaste.seconds()*2));
        } else if (timeWaste.seconds() > 3.8 && timeWaste.seconds() < 4.5){
            drive.holdPosition(55, 22, -Math.PI/2, .75);
            if (timeWaste.seconds() < 4.3){
                intake.setState(EXTENDEDACTIVE);
                intake.setSlidesLengthInches(16);
            } else {
                intake.setState(EXTENDEDREVERSED);
            }

        } else if (timeWaste.seconds() > 4.5 && timeWaste.seconds() < 5.2){
            drive.holdPosition(86, 22, -Math.PI/2, .75);
            intake.setState(HOMEUP);
        } else if (timeWaste.seconds() > 5.2 && timeWaste.seconds() < 5.9){
            drive.holdPosition(88, 9.5, -Math.PI/2, .75);}
        else if (timeWaste.seconds() > 5.9 && timeWaste.seconds() < 6.8){
            drive.holdPosition(60, 9.5, -Math.PI/2, .75);
            if (timeWaste.seconds() < 6.1) {
                intake.setState(EXTENDEDACTIVE);
                intake.setSlidesLengthInches(16);
            } else {
                intake.setState(EXTENDEDREVERSED);
            }

        } else if (timeWaste.seconds() > 6.8&& timeWaste.seconds() < 7.3) {
            if (timeWaste.seconds() < 7){
                drive.holdPosition(75.63, 20, -Math.PI/2, .75);
            } else {
                drive.holdPosition(75.63, 20, -0.7135, .75);}
            if (timeWaste.seconds() > 6.9){
                intake.setState(EXTENDEDACTIVE);
                intake.setSlidesLengthInches(2);
            } else {
                intake.setState(HOMEUP);
            }
        } else if (timeWaste.seconds() > 7.3 && timeWaste.seconds() < 8){
            drive.holdPosition(65, 20, -0.7135, .5);
        } else if (timeWaste.seconds() > 8 && timeWaste.seconds() < 8.8){
            drive.holdPosition(53, 17.25, -1.34, .75);
            intake.setSlidesLengthInches(5);
            if (timeWaste.seconds() > 8.6){
                intake.setState(EXTENDEDREVERSED);
            }
        } else if (timeWaste.seconds() > 7.8&& timeWaste.seconds() < 8.9){
            scoring.setState(INTAKESPECIMENREADY);
            intake.setState(HOMEUP);
            drive.holdPosition(20, 2, Math.PI/2, .75);
        } else if (timeWaste.seconds() > 8.9){
            setState(Intake1);
        }
    }

    public void intake1(){
        drive.holdPosition(32, 17, -Math.PI/2, .5);

        if(timeWaste.seconds() > 1 && !done1) {
            scoring.setState(INTAKESPECIMENFRONT);
            done1 = true;
        } else if(timeWaste.seconds() > 1.3){
            setState(Score3);
        }

    }

    public void score2(){
        if (timeWaste.seconds() < 1){
            drive.holdPosition(50, 72, -Math.PI/2, 1);
        }
        else{
            drive.holdPosition(78, 72,-Math.PI/2, 1);
        }if (timeWaste.seconds() > 2){
            cycles++;
            if (cycles == 5){
                setState(StopState);
            } else {
                setState(Intake2);
            }
        }else if(timeWaste.seconds()>1.8){
            if(!done3){
                scoring.setState(HIGHRUNGSCOREBACK);
                done3 = true;
            }
        }
    }

    public void scoreFirstSpike(){
        if (timeWaste.seconds() < 1.1){
            drive.holdPosition(55, 72, -Math.PI/2, 1);
        }
        else{
            drive.holdPosition(78, 72,-Math.PI/2, 1);
        }
        if (timeWaste.seconds() > 2.1){
            cycles++;
            setState(Intake2);
        }else if(timeWaste.seconds()>1.9){
            if(!done3){
                scoring.setState(HIGHRUNGSCOREBACK);
                done3 = true;
            }
        }
    }
    public void intake2(){
        if(timeWaste.seconds() < 1.5 && timeWaste.seconds() > .3) {
            drive.holdPosition(45, 31, -Math.PI/2,1);
        } else if(timeWaste.seconds() > 1.5) {
            drive.holdPosition(32, 31, -Math.PI/2, 1);
        } else if(timeWaste.seconds() < .3){
            drive.holdPosition(68, 72, -Math.PI/2, 1);
        }

        if (timeWaste.seconds() > .3 && !done2){
            scoring.setState(INTAKESPECIMENFRONTREADY);
            done2 = true;

        } else if (timeWaste.seconds() < 1.6 && timeWaste.seconds() > 2.15){
            scoring.setState(INTAKESPECIMENFRONT);
        }
        else if(timeWaste.seconds() > 1.8){
            setState(Score2);
        }
    }


    public void stopState(){
        if (timeWaste.seconds() > .3){
            drive.holdPosition(50, 72, -Math.PI/2, .8);}
        else {drive.holdPosition(32, 31, -Math.PI/2, 1);}
        intake.setState(HOMEUP);
        scoring.setState(HOME);
    }

    public void setState(AAA_Paths.Path state){
        //visionPortal.setActiveCamera(camera == 1 ? webcam1:webcam2);
        this.state = state;
        timeWaste.reset();
        done1 = false;
        done2 = false;
        done3 = false;
    }
}


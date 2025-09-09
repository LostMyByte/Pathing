
package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous;



import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.AAA_Paths.Path.Intake1;
import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.AAA_Paths.Path.Intake2;
import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.AAA_Paths.Path.PushSpikes;
import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.AAA_Paths.Path.Score1;
import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.AAA_Paths.Path.Score2;
import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.AAA_Paths.Path.StopState;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.MainIntake.IntakeStates.EXTENDEDACTIVE;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.MainIntake.IntakeStates.EXTENDEDUP;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.MainIntake.IntakeStates.HOMEUP;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Scoring.ScoringStates.HIGHRUNG;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Scoring.ScoringStates.HIGHRUNGSCORE;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Scoring.ScoringStates.HOME;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Scoring.ScoringStates.INTAKESPECIMEN;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Scoring.ScoringStates.INTAKESPECIMENREADY;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Scoring.ScoringStates.STARTAUTO;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.teamcode.KCP.Localization.TwoWheelOdometry;
import org.firstinspires.ftc.teamcode.teamcode.KCP.Movement;

import org.firstinspires.ftc.teamcode.teamcode.Subsystems.MainIntake;

@Autonomous(name = "SpecimenAuto5")
public class SpecimenAuto5 extends BaseOpMode {

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
        Constants.startAngle = Double.NaN;
        Scoring.encoderPosition = 0;
        drive = new Movement(Constants.halfRobotSizeX, 62.5,targetHeading);
        
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
            case PushSpikes:
                pushSpikes();
                break;
            case Intake1:
                intake1();
                break;
            case StopState:
                stopState();
                break;
        }
    }

    public void Score1() {
        if(!done1){
            scoring.setState(HIGHRUNG);
            done1 = true;
        }

        drive.holdPosition(45, 67,targetHeading, .45);
        //cycles++;
        if (timeWaste.seconds() > 1.7){
            scoring.setState(HOME);
            setState(PushSpikes);

        }else if(timeWaste.seconds()>1.5){
            if(!done3){
                scoring.setState(HIGHRUNGSCORE);
                done3 = true;
            }
        }

    }

    public void pushSpikes(){
        if(timeWaste.seconds() < .5){
            drive.holdPosition(30, 70, Math.PI/2, .7);
        } else if(timeWaste.seconds() < 1.2 && timeWaste.seconds() > .5) {
            drive.holdPosition(26, 40, Math.PI/2, .9);
        } else if (timeWaste.seconds() > 1.2 && timeWaste.seconds() < 2){
            drive.holdPosition(50, 40, Math.PI/2, .75);
        } else if (timeWaste.seconds() > 2 && timeWaste.seconds() < 2.7){
            drive.holdPosition(56, 25, Math.PI/2, .75);
        } else if (timeWaste.seconds() > 2.7 && timeWaste.seconds() < 3.9){
            drive.holdPosition(20, 25, Math.PI/2, .65);
        } else if (timeWaste.seconds() > 4 && timeWaste.seconds() < 4.4){
            drive.holdPosition(35, 30, Math.PI/2, .75);}
        else if (timeWaste.seconds() > 4.4 && timeWaste.seconds() < 5.6){
            drive.holdPosition(51, 18, Math.PI/2, .75);
        } else if (timeWaste.seconds() > 5.6 && timeWaste.seconds() < 6.6) {
            drive.holdPosition(20, 18, Math.PI / 2, .65);
        } else if (timeWaste.seconds() > 6.6 && timeWaste.seconds() < 7.4){
            drive.holdPosition(50, 20, Math.PI/2, .75);
        } else if (timeWaste.seconds() > 7.4 && timeWaste.seconds() < 8){
            drive.holdPosition(59, 2, Math.PI/2, .75);
        } else if (timeWaste.seconds() > 8 && timeWaste.seconds() < 9.1){
            scoring.setState(INTAKESPECIMENREADY);
            drive.holdPosition(20, 2, Math.PI/2, .75);
        } else if (timeWaste.seconds() > 9.1){
            setState(Intake1);
        }
    }

    public void moveSpikes(){
        if (timeWaste.seconds() > 8){
            drive.holdPosition(31,46, Math.toRadians(-45));
            intake.setState(EXTENDEDUP);
            intake.setSlidesLengthInches(17);
        }  else if(timeWaste.seconds() > 4){
            intake.setState(EXTENDEDACTIVE);
            intake.setSlidesLengthInches((timeWaste.seconds()-1.2) * 2 + 6.5);
        } else if(timeWaste.seconds() < 8){
            drive.holdPosition(31,46, Math.toRadians(45));
        }
    }

    public void intake1(){
        if(timeWaste.seconds() < 1) {
            drive.holdPosition(20, 34, Math.PI/2,.7);
        } else if(timeWaste.seconds() > 1) {
            drive.holdPosition(0, 34, Math.PI/2, .35);
        }
        if(timeWaste.seconds() > 2 && !done1) {
            scoring.setState(INTAKESPECIMEN);
            done1 = true;
        } else if(timeWaste.seconds() > 2.2){
            setState(Score2);
        }

    }

    public void score2(){
        drive.holdPosition(45, 70 + cycles*2.5,targetHeading, .45);
        if (timeWaste.seconds() > 2){
            cycles++;
            scoring.setState(HOME);
            if (cycles == 4){
                setState(StopState);
            } else {
                setState(Intake2);
            }
        }else if(timeWaste.seconds()>1.7){
            if(!done3){
                scoring.setState(HIGHRUNGSCORE);
                done3 = true;
            }
        }
    }
    public void intake2(){
        if(timeWaste.seconds() < 1.8 && timeWaste.seconds() > .3) {
            drive.holdPosition(20, 36, Math.PI/2,.7);
        } else if(timeWaste.seconds() > 1.8) {
            drive.holdPosition(3.5, 36, Math.PI/2, .5);
        } else if(timeWaste.seconds() < .3){
            drive.holdPosition(30, 70, Math.PI/2, .7);
        }

        if (timeWaste.seconds() > .3 && !done2){
            scoring.setState(INTAKESPECIMENREADY);
            done2 = true;
        } else if(timeWaste.seconds() > 2.5){
            scoring.setState(INTAKESPECIMEN);
            setState(Score2);
        }
    }


    public void stopState(){
        if (timeWaste.seconds() > .3){
            drive.holdPosition(5, 30, Math.PI/2, .8);}
        else {drive.holdPosition(30, 70, Math.PI/2, 1);}
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


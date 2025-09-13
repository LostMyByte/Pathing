
package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous;



import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.AAA_Paths.Path.FarSpike;
import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.AAA_Paths.Path.GoToScore;
import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.AAA_Paths.Path.HoldPark;
import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.AAA_Paths.Path.Lupine;
import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.AAA_Paths.Path.Score1;
import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.AAA_Paths.Path.Score2;
import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.AAA_Paths.Path.MiddleSpike;
import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.AAA_Paths.Path.Sub2;
import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.AAA_Paths.Path.TransferFail;
import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.AAA_Paths.Path.WallSpike;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.MainIntake.IntakeStates.EXTENDEDACTIVE;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.MainIntake.IntakeStates.EXTENDEDUP;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.MainIntake.IntakeStates.HOMEUP;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.MainIntake.IntakeStates.HOMEUPIN;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Scoring.ScoringStates.HIGHBUCKET;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Scoring.ScoringStates.HIGHBUCKETSCORE;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Scoring.ScoringStates.HOME;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Scoring.ScoringStates.POSTTRANSFER;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Scoring.ScoringStates.STARTAUTO;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Scoring.ScoringStates.STARTAUTOSAMPLE;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Scoring.ScoringStates.TRANSFER;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.teamcode.KCP.Localization.TwoWheelOdometry;
import org.firstinspires.ftc.teamcode.teamcode.KCP.Movement;

import org.firstinspires.ftc.teamcode.teamcode.Subsystems.MainIntake;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "BucketAutoLupine")
@Disabled
public class BucketAutoLupine extends BaseOpMode {

    Movement drive;
    Scoring scoring;
    MainIntake intake;
    AAA_Paths.Path state = Score1;
    ElapsedTime timeWaste = new ElapsedTime();
    ElapsedTime totalTime;
    AprilTagProcessor aprilTags2;
    double x;
    double y;
    boolean readyToDrive = false;

    //DO NOT SET TESTBED CHASSIS GLOBAL POWER LOWER THAN .55
    double globalPower = .55;
    double globalHeading = 0;
    int cycles = 0;
    boolean done1 = true;
    boolean done2 = true;
    boolean done3 = true;
    boolean done4 = true;

    @Override
    public void externalInit() {
        drive = new Movement(Constants.halfRobotSizeX + 1, 109,globalHeading);

        telemetry.update();
        totalTime = new ElapsedTime();
        timeWaste = new ElapsedTime();

        totalTime.reset();

        intake = new MainIntake(hardware);
        intake.setState(HOMEUP);
        scoring = new Scoring(hardware);
        scoring.setState(STARTAUTO);
        drive.setHeading(globalHeading);
        drive.setActiveCorrectionMethod(Movement.CorrectionMethods.Profiled);



        BaseOpMode.addData("X", TwoWheelOdometry.x());
        BaseOpMode.addData("Y", TwoWheelOdometry.y());
        BaseOpMode.addData("Heading", TwoWheelOdometry.heading());

    }

    @Override
    public void externalInitLoop() {
        intake.resetSlides();
        intake.setState(MainIntake.IntakeStates.HOMEUP);
        scoring.setState(STARTAUTOSAMPLE);
        BaseOpMode.addData("timeWaste", timeWaste.seconds());
        BaseOpMode.addData("State", state);
        BaseOpMode.addData("X", TwoWheelOdometry.x());
        BaseOpMode.addData("Y", TwoWheelOdometry.y());
        BaseOpMode.addData("H", TwoWheelOdometry.heading());
        BaseOpMode.addData("targetHeading", globalHeading);
    }

    @Override
    public void externalStart() {
        timeWaste.reset();
        totalTime.reset();
        drive.setHeading(globalHeading);
    }

    @Override
    public void externalLoop() {
        stateMachine();
        BaseOpMode.addData("timeWaste", timeWaste.seconds());
        BaseOpMode.addData("State", state);
        BaseOpMode.addData("X", TwoWheelOdometry.x());
        BaseOpMode.addData("Y", TwoWheelOdometry.y());
        BaseOpMode.addData("H", TwoWheelOdometry.heading());
        BaseOpMode.addData("targetHeading", globalHeading);
    }

    @Override
    public void externalStop() {
        Constants.startAngle = Math.PI/2;
    }

    public void stateMachine() {
        switch (state) {
            case Score1:
                Score1();
                break;
            case MiddleSpike:
                middleSpike();
                break;
            case Score2:
                score2();
                break;
            case FarSpike:
                farSpike();
                break;
            case WallSpike:
                wallSpike();
                break;
            case HoldPark:
                park();
                break;
            case Submersible:
                submersible();
                break;
            case Sub2:
                break;
            case ScoreFailure:
                break;
            case TransferFail:
                transferFail();
                break;
            case GoToScore:
                goToScore();
                break;
            case Lupine:
                lupine();
                break;
        }
    }

    public void transferFail(){
        drive.holdPosition(19, 137,globalHeading, globalPower);

        if (timeWaste.seconds() < .7){
            intake.setState(EXTENDEDACTIVE);
            intake.setSlidesLengthInches(7);
        } else if (timeWaste.seconds() > .7){
            intake.setState(HOMEUP);}
        if (timeWaste.seconds() > 1.2 && done1){
            scoring.setState(TRANSFER);
            scoring.stopResetting();
            done1 = false;
        } else if (timeWaste.seconds() < 1.2 && done2){
            scoring.setState(HOME);
            scoring.resetSlides();
            done2 = false;
        }
        if (!intake.checkForSample() && scoring.getState() == POSTTRANSFER && timeWaste.seconds() > 2.2){
            setState(Score2);
        } else if (scoring.getState() == POSTTRANSFER && intake.checkForSample() && timeWaste.seconds() > 2.2){
            setState(TransferFail);

        }
    }

    public void lupine(){
        drive.holdPosition(Constants.halfRobotSizeX+6.5,y,0, globalPower);
        intake.setState(MainIntake.IntakeStates.EXTENDEDACTIVE);
        intake.setSlidesLengthInches(0);
        if (timeWaste.seconds() > 1.2){
            globalPower = .5;
        } else {
            globalPower = .9;
        }

        if (timeWaste.seconds() > 1.7) {y -= 1;
        } else {y=70;}
        if (intake.getIntakeColor() == MainIntake.SampleColor.YELLOW){
            setState(GoToScore);
        } if (timeWaste.seconds() > 3.3){
            intake.setSlidesLengthInches(0);
            setState(GoToScore);
        }

    }

    public void Score1() {
        drive.holdPosition(Constants.halfRobotSizeX, 123,Math.toRadians(15), .7);

        if (done2 && timeWaste.seconds() < .3) {
            scoring.setState(HIGHBUCKET);
            done2 = false;
        }
        if (timeWaste.seconds() > 1.5 && done1){
            scoring.setState(HIGHBUCKETSCORE);
            done1 = false;
        } else if(timeWaste.seconds() > 2){
            setState(Lupine);
        }
    }

    public void middleSpike(){

        drive.holdPosition(x, 114,Math.PI/2, .55);
        intake.setState(MainIntake.IntakeStates.EXTENDEDACTIVE);

        if (timeWaste.seconds() > .5){intake.setSlidesLengthInches(6);}
        if (timeWaste.seconds() > 1.4) { x+=.5;
        } else {x=10;}
        if (intake.getIntakeColor() == MainIntake.SampleColor.YELLOW){
            setState(GoToScore);
        } if (timeWaste.seconds() > 3){
            intake.setState(HOMEUP);
            cycles++;
            setState(WallSpike);
        }



    }

    public void score2() {
        drive.holdPosition(4, 121, globalHeading, .55);
        globalHeading = Math.toRadians(45);

        if (done2 && timeWaste.seconds() > .3) {
            scoring.setState(HIGHBUCKET);
            done2 = false;
        }
        if (timeWaste.seconds() > 1.5 && done1) {
            scoring.setState(HIGHBUCKETSCORE);
            cycles++;
            done1 = false;
        } else if (timeWaste.seconds() > 1.7) {
            if (cycles == 1){
                setState(FarSpike);
            } else if (cycles == 2) {
                setState(MiddleSpike);
            } else if (cycles == 3) {
                setState(WallSpike);
            } else if (cycles == 4) {
                setState(HoldPark);
            }
        }
    }


    public void goToScore(){
        drive.holdPosition(8, 121, globalHeading, .8);
        if (cycles == 0){
        globalHeading = 0;
        } else {globalHeading = Math.PI/4;}

        if (done3) {
            intake.setState(HOMEUPIN);
            done3 = false;
        }
        if (timeWaste.seconds() > .7 && done4){
            intake.setState(HOMEUP);
            scoring.setState(TRANSFER);
            done4 = false;
        }
        if (timeWaste.seconds() > 1.45 && scoring.getState() == POSTTRANSFER)
            if (intake.checkForSample()){
                setState(TransferFail);
            } else {
                setState(Score2);
            }
    }

    public void farSpike(){

        drive.holdPosition(x, 105,Math.PI/2, .55);
        intake.setState(MainIntake.IntakeStates.EXTENDEDACTIVE);

        if (timeWaste.seconds() > .5){intake.setSlidesLengthInches(10);}
        if (timeWaste.seconds() > 1.3) { x+=.5;
            } else {x=7;}
        if (intake.getIntakeColor() == MainIntake.SampleColor.YELLOW){
            setState(GoToScore);
        } if (timeWaste.seconds() > 3.5){
            intake.setState(HOMEUP);
            cycles++;
            setState(MiddleSpike);
        }



    }


    public void wallSpike(){

        drive.holdPosition(24,104,globalHeading, .65);
        if (timeWaste.seconds() > 1.7){;
            intake.setState(MainIntake.IntakeStates.EXTENDEDACTIVE);
            intake.setSlidesLengthInches((timeWaste.seconds()-1.7) * 7);
        }
        if (timeWaste.seconds() > 2.1) { globalHeading = (3*Math.PI/4)-(timeWaste.seconds() - 2.1)/8;;
        } else if (timeWaste.seconds() > .5 && timeWaste.seconds() < 2.1) {
            globalHeading = 3*Math.PI/4;
        } else {globalHeading = Math.PI/4;}
        if (intake.getIntakeColor() == MainIntake.SampleColor.YELLOW){
            setState(GoToScore);
        } if (timeWaste.seconds() > 6){
            setState(HoldPark);
        }
    }

    public void park(){
        drive.holdPosition(40, 100,globalHeading, globalPower);
        if(done1) {
            intake.setState(HOMEUP);
            scoring.setState(HOME);
            done1 = false;
        }
        if(timeWaste.seconds() > .5){
            globalHeading = Math.PI/2;
        }
    }

    public void submersible(){
        globalHeading = 0;
        if (timeWaste.seconds() < 2) {
            drive.holdPosition(Constants.halfRobotSizeX + 70, 125, globalHeading, 1);
        } else {
            drive.holdPosition(Constants.halfRobotSizeX + 70, 109,globalHeading, 1);
        }
        if (timeWaste.seconds() > 2.5){
            intake.setState(EXTENDEDUP);
            intake.setSlidesLengthInches(3);
        }
        if (timeWaste.seconds() > 3){
            intake.setState(EXTENDEDACTIVE);
            intake.setSlidesLengthInches(15);
        }
        if (intake.getIntakeColor() == MainIntake.SampleColor.BLUE || intake.getIntakeColor() == MainIntake.SampleColor.YELLOW){
            setState(GoToScore);
        }
        if (intake.getIntakeColor() == MainIntake.SampleColor.RED || timeWaste.seconds() > 4) {
            setState(Sub2);
        }
    }


    public void setState(AAA_Paths.Path state){
        //visionPortal.setActiveCamera(camera == 1 ? webcam1:webcam2);
        this.state = state;
        timeWaste.reset();
        readyToDrive = false;
        done1 = true;
        done2 = true;
        done3 = true;
        done4 = true;
    }
}
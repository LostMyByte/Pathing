
package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous;



import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.AAA_Paths.Path.FarSpike;
import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.AAA_Paths.Path.GoToScore;
import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.AAA_Paths.Path.HoldPark;
import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.AAA_Paths.Path.Score1;
import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.AAA_Paths.Path.Score2;
import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.AAA_Paths.Path.MiddleSpike;
import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.AAA_Paths.Path.Submersible;
import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.AAA_Paths.Path.TransferFail;
import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.AAA_Paths.Path.WallSpike;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.MainIntake.IntakeStates.EXTENDEDACTIVE;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.MainIntake.IntakeStates.EXTENDEDNOTACTIVE;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.MainIntake.IntakeStates.HOMEUP;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.MainIntake.IntakeStates.HOMEUPIN;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.MainIntake.IntakeStates.HOMEUPOUT;

import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.MainIntake.IntakeStates.SEARCHJUSTYELLOW;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Scoring.ScoringStates.HIGHBUCKET;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Scoring.ScoringStates.HIGHBUCKETSCORE;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Scoring.ScoringStates.HOME;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Scoring.ScoringStates.POSTTRANSFER;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Scoring.ScoringStates.STARTAUTO;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Scoring.ScoringStates.STARTAUTOSAMPLE;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Scoring.ScoringStates.TRANSFER;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.teamcode.KCP.Localization.TwoWheelOdometry;
import org.firstinspires.ftc.teamcode.teamcode.KCP.Movement;

import org.firstinspires.ftc.teamcode.teamcode.Subsystems.MainIntake;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "BucketAuto")
public class BlueCloseBucketAuto extends BaseOpMode {

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
        Constants.startAngle = Double.NaN;
        drive = new Movement(-4.82, 95.39,1.5644);
        Scoring.encoderPosition = 0;
        Constants.startAngle = 1.5644;
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

        Constants.team = Constants.Team.BLUE;

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

        //drive.setHeading(1.5644);
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
        if (totalTime.seconds() > 29.5) {
            intake.setState(HOMEUP);
            scoring.setState(HOME);
        }
    }

    public void transferFail(){

        drive.holdPosition(4.7, 119.45, 1.4737, .6);

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
        drive.holdPosition(6.9045, 119.5,1.17, .9);

        if (done2 && timeWaste.seconds() < .3) {
            scoring.setState(HIGHBUCKET);
            done2 = false;
        }
        if (timeWaste.seconds() > 1){
            intake.setState(EXTENDEDNOTACTIVE);
            intake.setSlidesLengthInches(11);
        }
        if (timeWaste.seconds() > 1.6 && done1){
            scoring.setState(HIGHBUCKETSCORE);
            done1 = false;
        } else if(scoring.getState() == HOME && timeWaste.seconds() > 1.65){
            setState(FarSpike);
        }
    }

    public void middleSpike(){

        drive.driveBlind(.3);
        intake.setState(MainIntake.IntakeStates.EXTENDEDACTIVE);
        if (intake.getIntakeColor() == MainIntake.SampleColor.YELLOW){
            setState(GoToScore);
        } if (timeWaste.seconds() > 1.4){
            intake.setState(HOMEUP);
            cycles++;
            setState(WallSpike);
        }



    }

    public void score2() {
        globalHeading = 1.43;
        drive.holdPosition(4.7, 119.45, globalHeading, .6);

        if (done2 && timeWaste.seconds() > .3) {
            scoring.setState(HIGHBUCKET);
            done2 = false;
        }
        if (timeWaste.seconds() > 0.8 && cycles == 0){
            intake.setState(EXTENDEDNOTACTIVE);
            intake.setSlidesLengthInches(14);
        }

        if (timeWaste.seconds() > 1.5 && done1) {
            scoring.setState(HIGHBUCKETSCORE);
            cycles++;
            done1 = false;
        } else if (timeWaste.seconds() > 1.55) {
            if (cycles == 1) {
                setState(MiddleSpike);
            } else if (cycles == 2) {
                setState(WallSpike);
            } else if (cycles == 3 || cycles == 4) {
                setState(Submersible);
            } else if (cycles == 5){
                setState(HoldPark);
            }
        }
    }


    public void goToScore(){
        drive.holdPosition(4.7, 119.45, 1.4737, .6);

        if (done3) {
            intake.setState(HOMEUPIN);
            done3 = false;
        }
        if (timeWaste.seconds() > .3 && done4 && cycles == 2){
            intake.setState(HOMEUP);
            scoring.setState(TRANSFER);
            done4 = false;
        } else if (timeWaste.seconds() > .75 && done4){
            intake.setState(HOMEUP);
            scoring.setState(TRANSFER);
            done4 = false;
        }
        if (scoring.getState() == POSTTRANSFER)
            setState(Score2);

    }
    public void farSpike(){

        drive.driveBlind(.25);
        intake.setState(MainIntake.IntakeStates.EXTENDEDACTIVE);
        if (intake.getIntakeColor() == MainIntake.SampleColor.YELLOW){
            setState(GoToScore);
        } if (timeWaste.seconds() > 1.3){
            intake.setState(HOMEUP);
            setState(GoToScore);
        }
    }


    public void wallSpike(){

        drive.holdPosition(x,106,globalHeading, globalPower);
        if (timeWaste.seconds() > 1.4){;
            intake.setState(MainIntake.IntakeStates.EXTENDEDACTIVE);
            intake.setSlidesLengthInches(3);
        }
        if (timeWaste.seconds() > 1.35) {
            x = 40;
            globalPower = .4;
        } else if (timeWaste.seconds() > .5 && timeWaste.seconds() < 2.1) {
            globalHeading = 3*Math.PI/4;
        } else {
            globalPower = .9;
            x = 22.5;
        }
        if (intake.getIntakeColor() == MainIntake.SampleColor.YELLOW){
            intake.setState(HOMEUPIN);
            setState(GoToScore);
        } if (timeWaste.seconds() > 4.5){
            intake.setState(HOMEUPOUT);
            cycles++;
            setState(Submersible);
        }
    }

    public void park(){
        drive.holdPosition(40, 100,Math.PI/2, .7);
        if(done1 && timeWaste.seconds() > 1) {
            intake.setState(HOMEUPOUT);
            scoring.setState(HOME);
            done1 = false;
        }
    }

    double holdY = 92;
    double holdX = 62;
    double oldOffset = 0;
    public void submersible(){
        MainIntake.SampleColor color = intake.getIntakeColor();
        globalHeading = 1.43;
        if (timeWaste.seconds() < 1.2) {
            drive.holdPosition(57, 105, globalHeading, 1);

        } else if (timeWaste.seconds() < 1.9){
            drive.holdPosition(62 , 92,0.5, .7);
            holdY = 92;
            oldOffset = intake.blockMovement;
        }

        if (timeWaste.seconds() > 0.2){
            globalHeading = 0;
        }



        if ((timeWaste.seconds()>2 && (drive.getVelocity()[1] > -0.5)&& holdY == 92) || (timeWaste.seconds() > 2.5 && holdY == 92)) {
            holdY = drive.getY() - 1;
            intake.sweeperOut();
            intake.setState(SEARCHJUSTYELLOW);


            done1 = false;
        }
        if (intake.getState() == SEARCHJUSTYELLOW) {
            if (intake.blockMovement != oldOffset) {
                oldOffset = intake.blockMovement;
                holdX += MainIntake.CameraMount.DriveAlpha*(drive.getX() + intake.blockMovement - holdX);
            }
            BaseOpMode.addData("Holding X", holdX);
            BaseOpMode.addData("Holding Y", holdY);
            drive.holdPosition(holdX, holdY, 0, 1);
        }
        if (intake.visionSampleReady){
            setState(GoToScore);
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
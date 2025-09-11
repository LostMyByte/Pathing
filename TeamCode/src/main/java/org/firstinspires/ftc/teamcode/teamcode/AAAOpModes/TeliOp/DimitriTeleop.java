package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.TeliOp;

import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Scoring.ScoringStates.CLIMBDOWN;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Scoring.ScoringStates.CLIMBUP;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Scoring.ScoringStates.HIGHBUCKET;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Scoring.ScoringStates.HIGHBUCKETSCORE;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Scoring.ScoringStates.HIGHRUNG;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Scoring.ScoringStates.HIGHRUNGSCORE;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Scoring.ScoringStates.HOME;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Scoring.ScoringStates.INTAKESPECIMEN;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Scoring.ScoringStates.INTAKESPECIMENREADY;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Scoring.ScoringStates.LOWBUCKET;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Scoring.ScoringStates.LOWBUCKETSCORE;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Scoring.ScoringStates.NOTHING;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Scoring.ScoringStates.POSTTRANSFER;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Scoring.ScoringStates.TRANSFER;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.KCP.Movement;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.MainIntake;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Scoring;

@TeleOp(name="Dimitri Teleop")
public class DimitriTeleop extends BaseOpMode{

    //Drivetrain drivetrain;
    Scoring scoring;
    Movement autoDrive;
    MainIntake intake;
    double turretAngle = 0;
    double slidesLength = 0;
    double slidesSensitivityConstant = .65;
    ElapsedTime sweeperTimer;
    TeleStates state = TeleStates.DRIVING;
    double cycles = 0;
    ElapsedTime timeWaste;
    boolean done1;


    @Override
    public void externalInit() {


        autoDrive = new Movement(0,0,0);
        BaseOpMode.addData("endAngle", Constants.startAngle);

        scoring = new Scoring(hardware);
        intake = new MainIntake(hardware);
        intake.setState(MainIntake.IntakeStates.NOTHING);
        scoring.setState(NOTHING);
        sweeperTimer = new ElapsedTime();
        timeWaste = new ElapsedTime();
    }

    public void externalInitLoop(){

    }
    public enum Team{
        RED, BLUE, NOTDECLARED
    }
    public enum TeleStates{
        DRIVING, CYCLEINTAKE, CYCLESCORE
    }
    @Override
    public void externalStart(){
        scoring.setState(HOME);
        autoDrive.stopDrive();
        intake.setState(MainIntake.IntakeStates.HOMEUP);
        scoring.resetSlides();
    }
    public void stateMachine() {
        switch (state) {
            case DRIVING:
                driving();
                break;
            case CYCLESCORE:
                cycleScore();
                break;
            case CYCLEINTAKE:
                cycleIntake();
                break;
            }
        }



    @Override
    public void externalLoop(){
        stateMachine();
    }

    public void driving(){
        double drive  = -driver1.leftStick.Y();
        double strafe = -driver1.leftStick.X();
        double turn;
        double speed  = 1;
        boolean lockHeading;

        if(driver1.rightTrigger.isPressed()){
            speed = 0.5;
        }
        if (!driver1.leftBumper.isToggled()){
            turn = driver1.rightStick.X();
            lockHeading = false;
        }
        else {
            turn = 0;
            lockHeading = true;
        }

        if(driver1.share.isPressed()){
            intake.resetSlides();
            scoring.resetSlides();
        } else {
            intake.stopResetting();
        }

        if (driver1.dpad_up.isTapped()) {
            autoDrive.setHeading(0);
        }

        autoDrive.drive(drive, strafe,turn,speed, lockHeading);

        if (intake.getState() == MainIntake.IntakeStates.HOMEUP && driver2.triangle.isTapped()){
            intake.setState(MainIntake.IntakeStates.EXTENDEDUP);
            slidesLength = 1.5;
            turretAngle = 0;
        } else if (intake.getState() == MainIntake.IntakeStates.HOMEDOWNNOTACTIVE && driver2.triangle.isTapped()){
            intake.setState(MainIntake.IntakeStates.EXTENDEDUP);
            slidesLength = 1.5;
            turretAngle = 0;
        } else if (driver2.square.isTapped()){
            intake.setState(MainIntake.IntakeStates.HOMEUP);
            turretAngle = 0;
        }

        else if(driver1.dpad_down.isTapped()){
            if(scoring.getState() == CLIMBUP){
                scoring.setState(CLIMBDOWN);
            } else {
                scoring.setState(CLIMBUP);
            }
        }
/*
        else if (intake.getState() == MainIntake.IntakeStates.EXTENDEDUP && driver2.rightTrigger.isPressed()){
            intake.setState(MainIntake.IntakeStates.EXTENDEDACTIVE);
        } else if (intake.getState() == MainIntake.IntakeStates.HOMEDOWNNOTACTIVE && driver2.rightTrigger.isPressed()){
            intake.setState(MainIntake.IntakeStates.HOMEACTIVE);
        } else if (intake.getState() == MainIntake.IntakeStates.EXTENDEDUP && driver2.leftTrigger.isPressed()){
            intake.setState(MainIntake.IntakeStates.EXTENDEDREVERSED);
        } else if (intake.getState() == MainIntake.IntakeStates.HOMEDOWNNOTACTIVE && driver2.leftTrigger.isPressed()){
            intake.setState(MainIntake.IntakeStates.HOMEREVERSED);
        } else if (intake.getState() == MainIntake.IntakeStates.EXTENDEDACTIVE && !driver2.rightTrigger.isPressed()) {
            intake.setState(MainIntake.IntakeStates.EXTENDEDUP);
        } else if (intake.getState() == MainIntake.IntakeStates.EXTENDEDREVERSED && !driver2.leftTrigger.isPressed()) {
            intake.setState(MainIntake.IntakeStates.EXTENDEDUP);
        } else if (intake.getState() == MainIntake.IntakeStates.HOMEACTIVE && !driver2.rightTrigger.isPressed()) {
            intake.setState(MainIntake.IntakeStates.HOMEDOWNNOTACTIVE);
        } else if (intake.getState() == MainIntake.IntakeStates.HOMEREVERSED && !driver2.leftTrigger.isPressed()) {
                intake.setState(MainIntake.IntakeStates.HOMEDOWNNOTACTIVE);
        }


 */

        if (intake.getState() == MainIntake.IntakeStates.EXTENDEDNOTACTIVE || intake.getState() == MainIntake.IntakeStates.EXTENDEDACTIVE){
            if (driver2.rightTrigger.isPressed()){
                intake.setState(MainIntake.IntakeStates.EXTENDEDACTIVE);
            } else if (driver2.leftBumper.isTapped()){
                intake.setState(MainIntake.IntakeStates.EXTENDEDUP);
            } else if (driver2.leftTrigger.isPressed()){
                intake.setState(MainIntake.IntakeStates.EXTENDEDREVERSED);
            } else {intake.setState(MainIntake.IntakeStates.EXTENDEDNOTACTIVE);}
        } else if (intake.getState() == MainIntake.IntakeStates.EXTENDEDUP && driver2.leftBumper.isTapped()){
            intake.setState(MainIntake.IntakeStates.EXTENDEDNOTACTIVE);
        } else if (driver2.leftTrigger.isPressed()){
            intake.setState(MainIntake.IntakeStates.EXTENDEDREVERSED);
        } else if (intake.getState() == MainIntake.IntakeStates.EXTENDEDREVERSED && !driver2.leftTrigger.isPressed()){
            intake.setState(MainIntake.IntakeStates.EXTENDEDUP);
        } else if (intake.getState() == MainIntake.IntakeStates.HOMEUP || intake.getState() == MainIntake.IntakeStates.HOMEUPIN || intake.getState() == MainIntake.IntakeStates.HOMEUPOUT){
            if (driver2.leftTrigger.isPressed()){
                intake.setState(MainIntake.IntakeStates.HOMEUPOUT);
            } else if (driver2.rightTrigger.isPressed()){
                intake.setState(MainIntake.IntakeStates.HOMEUPIN);
            } else if (driver2.cross.isTapped()){
                scoring.setState(TRANSFER);
            } else{
                intake.setState(MainIntake.IntakeStates.HOMEUP);
            }
        }

        if (driver1.circle.isTapped() && (scoring.getState() != INTAKESPECIMEN && scoring.getState() != INTAKESPECIMENREADY)){
            scoring.setState(HOME);
        }

        else if (driver1.square.isTapped() && (scoring.getState() == INTAKESPECIMEN  || scoring.getState() == POSTTRANSFER || scoring.getState() == INTAKESPECIMENREADY || scoring.getState() == HOME)){
            if (scoring.getState() == INTAKESPECIMENREADY){
                //intake and go to high rung
                scoring.setState(INTAKESPECIMEN);}
            else if (scoring.getState() != INTAKESPECIMEN){
                //from any other position(except intaking) prepare to intake
                scoring.setState(INTAKESPECIMENREADY);
            }
        } else if (driver1.rightBumper.isTapped() && scoring.getState() != TRANSFER){
            scoring.setState(HIGHBUCKET);
        } else if (driver1.cross.isTapped() && scoring.getState() != TRANSFER){
            scoring.setState(LOWBUCKET);
        }

        else if (driver1.square.isTapped()){
            if (scoring.getState() == HIGHBUCKET){
                scoring.setState(HIGHBUCKETSCORE);
            } else if (scoring.getState() == LOWBUCKET){
                scoring.setState(LOWBUCKETSCORE);
            } else if (scoring.getState() == HIGHRUNG){
                scoring.setState(HIGHRUNGSCORE);
            } else if (scoring.getState() == HIGHRUNGSCORE){
                scoring.setState(INTAKESPECIMENREADY);
            }
        }
        else if (driver1.dpad_up.isTapped() && scoring.getState() == HIGHRUNGSCORE){
            scoring.setState(HIGHRUNG);
        }

        if (intake.getState() == MainIntake.IntakeStates.EXTENDEDACTIVE || intake.getState() == MainIntake.IntakeStates.EXTENDEDREVERSED  || intake.getState() == MainIntake.IntakeStates.EXTENDEDUP || intake.getState() == MainIntake.IntakeStates.EXTENDEDNOTACTIVE){
            if(driver2.dpad_up.isPressed()){
                slidesLength += slidesSensitivityConstant;
            } else if(driver2.dpad_down.isPressed()){
                slidesLength -= slidesSensitivityConstant;
            }
            slidesLength = Range.clip(slidesLength, 0,17);

            intake.setSlidesLengthInches(slidesLength);
            BaseOpMode.addData("'SlidesLength'", slidesLength);
            BaseOpMode.addData("controller", driver2.leftStick.Y());
        }

        if ((driver2.circle.isPressed() || sweeperTimer.seconds() < .4) && ((intake.getState() != MainIntake.IntakeStates.EXTENDEDACTIVE || intake.getState() != MainIntake.IntakeStates.EXTENDEDNOTACTIVE || intake.getState() != MainIntake.IntakeStates.EXTENDEDREVERSED) || (intake.getSlidesLengthInches() > 5))){
            intake.sweeperOut();
        } else {
            intake.sweeperIn();
        }

        if (driver2.circle.isPressed()){
            sweeperTimer.reset();
        }

        if(driver1.playstation.isTapped()){
            autoDrive.setHeading(Math.PI/2);
            autoDrive.setPosition(45, 90);
            setState(TeleStates.CYCLEINTAKE);
        }


        BaseOpMode.addData("Slides Current", scoring.getSlidesCurrent());
        BaseOpMode.addData("Intake State", intake.getState());
        //BaseOpMode.addData("Scoring State", scoring.getState());
        BaseOpMode.addData("Slides Length", intake.getSlidesLengthInches());
        BaseOpMode.addData("V Slides Length", scoring.getSlidesHeight());
        BaseOpMode.addData("V Slides Inches", scoring.getSlidesHeightInches());
        BaseOpMode.addData("Intake State", intake.getState());
        BaseOpMode.addData("drive", driver1.leftStick.Y());
        BaseOpMode.addData("strafe", driver1.leftStick.X());
        BaseOpMode.addData("slides Length Inches", slidesLength);


    }

    public void cycleScore(){
        autoDrive.holdPosition(45, 73 + cycles*2,Math.PI/2, .45);
        if (timeWaste.seconds() > 1.9){
            cycles++;
            scoring.setState(HOME);
            setState(TeleStates.CYCLEINTAKE);

        }else if(timeWaste.seconds()>1.7){
            if(!done1){
                scoring.setState(HIGHRUNGSCORE);
                done1 = true;
            }
        }

        if(driver1.playstation.isTapped()){
            setState(TeleStates.DRIVING);
        }
    }

    public void cycleIntake(){
        if(timeWaste.seconds() < 1.8 && timeWaste.seconds() > .3) {
            autoDrive.holdPosition(20, 36, Math.PI/2,.7);
        } else if(timeWaste.seconds() > 1.8) {
            autoDrive.holdPosition(3.5, 36, Math.PI/2, .5);
        } else if(timeWaste.seconds() < .3){
            autoDrive.holdPosition(30, 70, Math.PI/2, .7);
        }

        if (timeWaste.seconds() > .3 && !done1){
            scoring.setState(INTAKESPECIMENREADY);
            done1 = true;
        } else if(timeWaste.seconds() > 2.5){
            scoring.setState(INTAKESPECIMEN);
            setState(TeleStates.CYCLESCORE);
        }
        if(driver1.playstation.isTapped()){
            setState(TeleStates.DRIVING);
        }
    }


    public void setState(TeleStates state){
        //visionPortal.setActiveCamera(camera == 1 ? webcam1:webcam2);
        this.state = state;
        timeWaste.reset();
        done1 = false;
    }


}

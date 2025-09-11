package org.firstinspires.ftc.teamcode.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Constants.slidesStartHeightConstant;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Constants.ticksToVSlidesInchesConstant;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Control.PID;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Dash.PIDTuningDash;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.HardwareDevices.Motor;

public class Scoring extends Subsystem{
    Motor vSlides;
    Motor vSlidesL;
    PID vSlidesPID;
    //Servos.Depositor depositorRotation;
    //Servos.SpecimenClaw specimenClaw;
    //Servos.DepositorDoor depositorDoor;
    double power;
    ScoringStates state = ScoringStates.HOME;
    ScoringStates lastState;
    NormalizedColorSensor colorSensor;
    ElapsedTime stateTime;
    Servos.DepositorArm depositorArm;
    Servos.DepositorArmLeft depositorArmLeft;
    Servos.DepositorClaw claw;
    Servos.DifferentialLeft differentialLeft;
    Servos.DifferentialRight differentialRight;
    boolean done1 = false;
    double localSlidesHeight = 0;
    public static double encoderPosition = 0;
    boolean slidesResetting = false;
    boolean climbing = false;
    TouchSensor limitSwitch;

    public Scoring(HardwareMap hardware){
        vSlidesL = new Motor(Hardware.VslideL, true);
        vSlides = new Motor(Hardware.VslideR, false,true);
        vSlides.pair(vSlidesL);
        depositorArm = new Servos.DepositorArm();
        depositorArmLeft = new Servos.DepositorArmLeft();
        differentialLeft = new Servos.DifferentialLeft();
        differentialRight = new Servos.DifferentialRight();
        claw = new Servos.DepositorClaw();
        //depositorRotation = new Servos.Depositor();
        //specimenClaw = new Servos.SpecimenClaw();
        //depositorDoor = new Servos.DepositorDoor();
        stateTime = new ElapsedTime();
        vSlidesPID = new PID(0,0,0);
        localSlidesHeight = encoderPosition;
        vSlides.encoder.setOffset(encoderPosition);
        //colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        limitSwitch = hardware.get(TouchSensor.class, "magnetSensor");

    }

    public void work(){
        switch (state){
            case HOME:
                home();
                break;
            case STARTAUTO:
                startAuto();
                break;
            case HIGHBUCKET:
                highBucket();
                break;
            case LOWBUCKET:
                lowBucket();
                break;
            case HIGHBUCKETSCORE:
                highBucketScore();
                break;
            case LOWBUCKETSCORE:
                lowBucketScore();
                break;
            case TRANSFER:
                transfer();
                break;
            case INTAKESPECIMENREADY:
                intakeSpecimenReady();
                break;
            case INTAKESPECIMEN:
                intakeSpecimen();
                break;
            case HIGHRUNG:
                highRung();
                break;
            case LOWRUNG:
                lowRung();
                break;
            case HIGHRUNGSCORE:
                highRungScore();
                break;
            case LOWRUNGSCORE:
                lowRungScore();
                break;
            case POSTTRANSFER:
                postTransfer();
                break;
            case CLIMBUP:
                climbUp();
                break;
            case CLIMBDOWN:
                climbDown();
                break;
            case STARTAUTOSAMPLE:
                startSampleAuto();
                break;
            case HIGHRUNGBACK:
                highRungBack();
                break;
            case HIGHRUNGSCOREBACK:
                highRungBackScore();
                break;
            case INTAKESPECIMENFRONTREADY:
                intakeSpecimenReadyFront();
                break;
            case INTAKESPECIMENFRONT:
                intakeSpecimenFront();
                break;
            case NOTHING:
                break;
        }

    }

    public void updateSlides(double target){
        if (slidesResetting){
            if (!limitSwitch.isPressed()){
                vSlides.setPower(-.8);
            } else {
                vSlides.setPower(0);
                slidesResetting = false;
            }
        } else {
            if (climbing) {
                vSlidesPID.setConstants(PIDTuningDash.VSlidesPCLIMB, 0,0);
            } else {
                vSlidesPID.setConstants(PIDTuningDash.VSlidesP, 0, PIDTuningDash.VSlidesD);}

            encoderPosition = vSlides.encoder.getPosition();
            BaseOpMode.addData("V Slides Position", encoderPosition);
            double correction = vSlidesPID.getCorrection(encoderPosition, target);
//        if(target == 0 && slides.encoder.getVelocity() > slidesConstantDownThresholdVelocity && slides.encoder.getPosition() > slidesConstantDownThresholdPosition) {
//            correction = slidesConstantDown;
//            slides.encoder.resetEncoder();
//        }

            vSlides.setPower(correction);
            BaseOpMode.addData("Target", target);
            BaseOpMode.addData("VCorrection", correction);
            BaseOpMode.addData("SlidesCurrent", vSlides.getCurrent());
//        BaseOpMode.addData("Current", -slides.encoder.getPosition());
        }
        if (limitSwitch.isPressed()){
            vSlides.encoder.resetEncoder();
        }
        BaseOpMode.addData("slidesResetting", slidesResetting);
        BaseOpMode.addData("limitSwitch", limitSwitch);
    }

    public void setSlidesHeight(double height){
        localSlidesHeight = height;
    }

    public double getSlidesHeight(){
        return vSlides.encoder.getPosition();
    }
    public double getSlidesHeightInches(){
        return  (getSlidesHeight()*ticksToVSlidesInchesConstant) + slidesStartHeightConstant;
    }

    public double getSlidesCurrent(){
        return vSlides.getCurrent();
    }

    public void setDepositorAngleRaw(double angle){
        depositorArm.setPositionRaw(angle);
        depositorArmLeft.setPositionRaw(angle);
    }
    public void setDepositorAngleInterpolated(double angle){
        depositorArm.setPositionInterpolated(angle);
        depositorArmLeft.setPositionInterpolated(angle);
    }




    //STATES
    public void home(){
        setSlidesHeight(0);
        setDepositorAngleInterpolated(30);
        if (stateTime.seconds() > .5){
            differentialRaw(.03,.57);
        }

        claw.open();

    }
    public void startAuto(){
        setSlidesHeight(0);
        setDepositorAngleInterpolated(20);
        if (stateTime.seconds() > .5){
            differentialRaw(.03,.57);
        }
        claw.closed();


    }

    public void startSampleAuto(){
        setSlidesHeight(0);
        setDepositorAngleInterpolated(30);
        differentialRaw(.03,.57);
        claw.closed();

    }

    public void climbUp(){
        setSlidesHeight(2000);
    }
    public void climbDown(){
        setSlidesHeight(900);
    }
    public void postTransfer(){
        setSlidesHeight(0);
        setDepositorAngleInterpolated(40);
        differentialRaw(.03,.57);
        claw.closed();

    }

    public void highBucketScore() {
        setSlidesHeight(2200);
        setDepositorAngleInterpolated(110);
        //fix
        differentialRaw(.85,.18);
        claw.open();
        if (stateTime.seconds() > .5){
            differentialRaw(.0,.6);
            setState(ScoringStates.HOME);
        }

    }

    //90 degrees claw rotation has specimens facing up and down
    public void differential(double wristAngle, double clawRotation){
        differentialLeft.setPosition(wristAngle + clawRotation);
        differentialRight.setPosition(clawRotation-wristAngle);
    }

    public void differentialRaw(double right, double left){
        differentialLeft.setPosition(left);
        differentialRight.setPosition(right);
    }

    public void resetSlides(){
        slidesResetting = true;
    }
    public void stopResetting(){
        slidesResetting = false;
    }

    public void startTeleop(){
        setSlidesHeight(-2500);
        setDepositorAngleInterpolated(30);
        if (stateTime.seconds() > .5){
            differentialRaw(.03,.57);
        }
        claw.open();
    }

    public void lowBucketScore() {
        setSlidesHeight(1050);
        if (getSlidesHeightInches() > 22) {
            if (!done1){
                done1 = true;
                stateTime.reset();
            }
            setDepositorAngleInterpolated(110);
            //fix
            differentialRaw(.85,.18);
            claw.open();
            if (stateTime.seconds() > .5){
                setState(ScoringStates.HOME);
            }
        }
    }

    public void highBucket(){
        setSlidesHeight(2200);
        claw.closed();
        if (getSlidesHeight() < 1800) {
            setDepositorAngleInterpolated(90);
            differentialRaw(.8,.6);
        } else {
            setDepositorAngleInterpolated(110);
            differentialRaw(.85,.18);
        }
    }

    public void lowBucket(){
        setSlidesHeight(1050);
        claw.closed();
        if (getSlidesHeight() < 900) {
            setDepositorAngleInterpolated(90);
            differentialRaw(.85,.58);
        } else {
            setDepositorAngleInterpolated(110);
            differentialRaw(.85,.18);
        }
    }

    public void intakeSpecimenReady(){
        if(stateTime.seconds() < 1){
            setSlidesHeight(600);
        } else {setSlidesHeight(300);}
        if(getSlidesHeight() > 275){
            setDepositorAngleInterpolated(-110);
            //fix
            differentialRaw(0.01,1);
        }
        claw.open();




    }
    public void intakeSpecimen(){
        setSlidesHeight(300);
        claw.closed();
        setDepositorAngleInterpolated(-110);
        differentialRaw(0.01,1);
        if(stateTime.seconds() > .5){
            setState(ScoringStates.HIGHRUNG);
        }
    }
    public void intakeSpecimenReadyFront(){
        if (stateTime.seconds() < .3 && lastState == ScoringStates.HOME){
            setSlidesHeight(200);
        } else {
        setSlidesHeight(-40);}
        claw.open();
        setDepositorAngleInterpolated(-30);
        differentialRaw(0.22,0.38);
    }
    public void intakeSpecimenFront(){
        setSlidesHeight(-40);
        claw.closed();
        setDepositorAngleInterpolated(-25);
        differentialRaw(0.22,0.38);
        if(stateTime.seconds() > .4){
            setState(ScoringStates.HIGHRUNGBACK);
        }
    }
    public void highRung() {
        claw.closed();
        if (stateTime.seconds() > .2){
            setDepositorAngleInterpolated(45);
            differentialRaw(.655, .805);
        }
        if (stateTime.seconds() < .5){
            setSlidesHeight(1000);
        } else {
        setSlidesHeight(700);
        }

    }

    public void highRungBackWorse(){
        claw.closed();
        setSlidesHeight(100);
        differentialRaw(.5,.1);
        setDepositorAngleInterpolated(130);
    }

    public void highRungBack(){
        claw.closed();
        setSlidesHeight(530);
        differentialRaw(.85,.18);
        setDepositorAngleInterpolated(77);
    }
    public void highRungBackScore(){
        claw.closed();
        setSlidesHeight(-40);
        differentialRaw(.85,.18);
        setDepositorAngleInterpolated(77);
    }


    public void lowRung(){
        if(stateTime.seconds() < 1){
            setSlidesHeight(900);
        } if(stateTime.seconds() > .5){
            setDepositorAngleInterpolated(-2);
            differentialRaw(.655,.805);
        } if (stateTime.seconds() > 1) {
            setSlidesHeight(0);
        }
        claw.closed();
    }
    public void highRungScore(){
        setSlidesHeight(0);
        setDepositorAngleInterpolated(30);
        differentialRaw(.655,.805);
        claw.closed();
    }
    public void lowRungScore() {
        setSlidesHeight(0);
        setDepositorAngleInterpolated(-30);
        differentialRaw(.655,.805);
        claw.closed();
    }

    public void transfer() {
        setDepositorAngleInterpolated(10);
        setSlidesHeight(0);
        differentialRaw(.03,.57);
        claw.closed();
        if(stateTime.seconds() > .45){
            setState(ScoringStates.POSTTRANSFER);
            BaseOpMode.addData("Checkpoint2","");
        }
        BaseOpMode.addData("checkpoint3", "");

    }

    public boolean slidesStalled(){
        //adjust this threshold
        if (vSlides.getPower() > .1 && vSlides.getVelocity() > 0){
            return true;
        } else {
            return false;
        }
    }


    public enum ScoringStates{
        HOME, HIGHBUCKET, LOWBUCKET, HIGHBUCKETSCORE, LOWBUCKETSCORE, INTAKESPECIMENREADY,
        INTAKESPECIMEN, INTAKESPECIMENFRONTREADY, INTAKESPECIMENFRONT, HIGHRUNG, HIGHRUNGBACK, HIGHRUNGSCOREBACK, LOWRUNG, HIGHRUNGSCORE, LOWRUNGSCORE, TRANSFER, POSTTRANSFER,
        TRANSFERTODROP, CLIMBUP, CLIMBDOWN, STARTAUTO, STARTAUTOSAMPLE, HIGHRUNGAUTO, NOTHING;
    }

    public void setState(ScoringStates state){
        lastState = getState();
        this.state = state;
        stateTime.reset();
        done1 = false;
    }
    public void update(){
        updateSlides(localSlidesHeight);
        work();
        if (getState() == ScoringStates.CLIMBDOWN){
            climbing = true;
        } else {
            climbing = false;
        }
    }
    public void updateSensors(){}

    public ScoringStates getState(){
        return state;
    }

}


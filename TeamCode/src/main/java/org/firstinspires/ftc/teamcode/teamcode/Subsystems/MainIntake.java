package org.firstinspires.ftc.teamcode.teamcode.Subsystems;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Control.PID;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Dash.PIDTuningDash;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.HardwareDevices.Motor;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Vision.LimelightBlockDetection;

public class MainIntake extends Subsystem{

    //will finish updating later
    Motor hSlides;
    CRServo intakeLeft;
    CRServo intakeRight;
    PID hSlidesPID;

    double correction;
    Servos.IntakeArmLeft intakeArmLeft;
    Servos.BottomSweeper sweeper;
    boolean transferReady = false;
    Servos.IntakeArmRight intakeArmRight;
    IntakeStates state = IntakeStates.HOMEUP;
    ElapsedTime stateTime;
    ElapsedTime sweeperTimer;
    double localSlidesLength = 0;
    DistanceSensor distanceSensor;
    NormalizedColorSensor intakeSensor;
    NormalizedRGBA colors;
    AnalogInput turretEncoder;
    boolean slidesResetting = false;
    final float[] hsvValues = new float[3];
    LimelightBlockDetection processor;
    boolean ready = false;



    public double blockMovement;



    @Config
    public static class CameraMount {
        public static double pitchAngle = 30;
        public static double yawAnlge = -20;
        public static double xOffset = -4.3;
        public static double yOffset = -0.5;
        public static double extraDistance = 3;
        public static boolean Activate = true;

        public static double distThreshold = 3;
        

        public static double Scalar = 0.1;
        public static double DriveScalar = 0.9;
        public static double DriveAlpha = 0.6;

    }

    @Config
    public static class intakeAngles {
        public static double intakeUpLeft = 0;
        public static double intakeUpRight = 0;

        public static double intakeDownLeft = 20;
        public static double intakeDownRight = 40;
    }

    public enum SearchMode {
        SlidesOnly,
        SlidesYDriveX,

    }

    public SearchMode activeMode = SearchMode.SlidesYDriveX;

    public MainIntake(HardwareMap hardwareMap){
//hSlides = new Motor(Hardware.extendo, false,true);
        hSlidesPID = new PID(0,0,0);
        intakeLeft = new com.arcrobotics.ftclib.hardware.motors.CRServo(hardwareMap,Hardware.intakeLeft);
        intakeRight = new com.arcrobotics.ftclib.hardware.motors.CRServo(hardwareMap,Hardware.intakeRight);
        intakeArmLeft = new Servos.IntakeArmLeft();
        intakeArmRight = new Servos.IntakeArmRight();
        sweeper = new Servos.BottomSweeper();

        processor = new LimelightBlockDetection();
        stateTime = new ElapsedTime();

        intakeSensor = hardwareMap.get(NormalizedColorSensor.class, "intakeSensor");
        distanceSensor = (DistanceSensor) intakeSensor;
    }
    public void work(){
        switch (state){
            case HOMEDOWNNOTACTIVE:
                homeDownNotActive();
                break;
            case HOMEUP:
                homeUp();
                break;
            case HOMEACTIVE:
                homeActive();
                break;
            case EXTENDEDACTIVE:
                extendedActive();
                break;
            case EXTENDEDUP:
                extendedUp();
                break;
            case EXTENDEDREVERSED:
                extendedReversed();
                break;
            case HOMEREVERSED:
                homeReversed();
                break;
            case EXTENDEDNOTACTIVE:
                extendedNotActive();
                break;
            case HOMEUPIN:
                homeUpIn();
                break;
            case HOMEUPOUT:
                homeUpOut();
                break;

            case SEARCHJUSTYELLOW:
                search(true, false);
                break;
            case SEARCHNOYELLOW:
                search(false, true);
                break;
            case SEARCHBOTH:
                search(true, true);
                break;
            case REJECT:
                reject();
                break;
        }
    }

    public void setTargetSlidesLength(double position){
        localSlidesLength = position;
    }

    public void updateSlides(){
        hSlidesPID.setConstants(PIDTuningDash.HSlidesP, 0, PIDTuningDash.HSlidesD);
        hSlidesPID.setFeedForward(PIDTuningDash.HSlidesF);
        //hSlidesPID.setLowerLimit(PIDTuningDash.HSlidesL);
        if (!Double.isNaN(getSlidesLengthInches())) {
            correction = hSlidesPID.getCorrection(getSlidesLengthInches(), getTargetSlidesLengthInches());
        }
        //limits the correction unless it is going to the home position
        if (localSlidesLength != -30){
            correction = Range.clip(correction, -.7, .7);}
        else {
            correction = Range.clip(correction, -1, -.1);
        }

        hSlides.setPower(correction);
        BaseOpMode.addData("slides target length", localSlidesLength);
        BaseOpMode.addData("Hcorrection", correction);

    }
    public void setSlidesLengthInches(double length){
        length = Range.clip(length, 0,17);
        localSlidesLength = (Math.asin((length-Constants.D)/Constants.A)+Constants.C)/Constants.B;
    }
    public void increaseSlidesLength(double length){
        localSlidesLength += length;
    }

    public void increaseSlidesLengthInches(double length){
        length = getTargetSlidesLengthInches() + length;
        setSlidesLengthInches(length);
    }

    public void sweeperOut(){
        sweeper.extended();
    }
    public void sweeperIn(){
        sweeper.retracted();
    }
    public double getSlidesLength(){
        return hSlides.encoder.getPosition();
    }
    public double getSlidesLengthInches(){ return Constants.A*Math.sin(Constants.B*hSlides.encoder.getPosition()-Constants.C)+Constants.D; }
    public void setArmAngle(double angle){
        intakeArmRight.setPositionInterpolated(angle);
        intakeArmLeft.setPositionInterpolated(angle);
    }
    public void intakeUp(){
        setArmAngle(97);
    }


    public void intakeMid(){
        setArmAngle(10);
    }
    public void intakeDown(){
        setArmAngle(-29+(getSlidesLengthInches()*.25));
    }
    public double getTargetSlidesLengthInches(){
        return Constants.A*Math.sin(Constants.B*localSlidesLength-Constants.C)+Constants.D;
    }
    public void resetSlides(){
        hSlides.setPower(-.2);
        hSlides.encoder.resetEncoder();
        slidesResetting = true;
    }
    public void stopResetting(){
        slidesResetting = false;
    }

    public void homeDownNotActive(){
        intakeDown();
        stopIntake();
        setTargetSlidesLength(0);
    }
    public void homeUp(){
        intakeUp();
        stopIntake();
        setTargetSlidesLength(-30);
    }
    public void homeActive(){
        intakeDown();
        runIntake();
        setTargetSlidesLength(0);
    }

    public void homeUpIn(){
        intakeUp();
        runIntake();
        setTargetSlidesLength(-30);
    }

    public void homeUpOut(){
        intakeUp();
        reverseIntake();
        setTargetSlidesLength(-30);
    }

    public void extendedActive(){
        intakeDown();
        runIntake();
        setTargetSlidesLength(localSlidesLength);
    }
    public void extendedNotActive(){
        intakeDown();
        stopIntake();
        setTargetSlidesLength(localSlidesLength);
    }

    public void extendedUp(){
        intakeMid();
        stopIntake();
        setTargetSlidesLength(localSlidesLength);
    }
    public void extendedReversed(){
        intakeDown();
        reverseIntake();
        setTargetSlidesLength(localSlidesLength);
    }
    public void homeReversed() {
        intakeDown();
        reverseIntake();
        setTargetSlidesLength(0);
    }

    ElapsedTime sampleInTimer = new ElapsedTime();
    boolean isSampleIn = false;
    public boolean visionSampleReady = false;
    IntakeStates visonState = IntakeStates.SEARCHJUSTYELLOW;

 // Offsets for driving:
    // use intake.blockMovement

    // Heading  Offset
    // 0        x + blockMovement
    // Pi/2     y + blockMovement
    // Pi       x - blockMovement
    // 3Pi/2    y - blockMovement
    // General  x + blockMovement*cos(theta), y + blockMovement*sin(theta)
    public void search(boolean yellow, boolean team){
        visonState = state;
        SampleColor color = getIntakeColor();
        if (color != SampleColor.EMPTY && !isSampleIn) {
            isSampleIn = true;
            sampleInTimer.reset();
        }
        if (stateTime.seconds() < 0.5) {
            sweeperOut();
            // Give it a second to reset slides


            processor.yMin = 15.5;
            sampleInTimer.reset();
            setSlidesLengthInches(2);
            reverseIntake();
            isSampleIn = false;
            visionSampleReady = false;
        }
        else if (stateTime.seconds() < 0.7) {
            // Give it a second to reset slides


            processor.yMin = 15.5;
            sampleInTimer.reset();
            setSlidesLengthInches(2);
            reverseIntake();
            isSampleIn = false;
            visionSampleReady = false;

            sweeperIn();
        }
        else if (stateTime.seconds() < 0.8) {
            // Give it a second to reset slides


            processor.yMin = 15.5;
            sampleInTimer.reset();
            setSlidesLengthInches(2);
            reverseIntake();
            isSampleIn = false;
            visionSampleReady = false;
            intakeDown();
            sweeperIn();
        }

        else if (isSampleIn && sampleInTimer.seconds() >= 0.15) {
            isSampleIn = false;
            sampleInTimer.reset();
            if (color == SampleColor.YELLOW) {
                setState(IntakeStates.HOMEUP);

                visionSampleReady = true;
                return;
            }
            else if (color == SampleColor.RED && team) {
                setState(IntakeStates.HOMEUP);
                visionSampleReady = true;
                return;
            }
            else {
                setState(IntakeStates.REJECT);
                return;
            }


        }

        else {

            processor.yMin = 16.5 - getSlidesLengthInches();

            double[] displacement = getDisplacement(yellow, team);

            if (displacement != null) {

                BaseOpMode.addData("Block Displacement X", displacement[0]);
                BaseOpMode.addData("Block Displacement Y", displacement[1]);

                double slidesLength = displacement[1] + CameraMount.yOffset + CameraMount.extraDistance;
                BaseOpMode.addData("slidesLength", slidesLength);

                if (CameraMount.Activate) {
                    switch (this.activeMode) {
                        case SlidesOnly:
                            increaseSlidesLengthInches(slidesLength * CameraMount.Scalar);

                            blockMovement = 0;
                            break;
                        case SlidesYDriveX:
                            if (Math.abs(displacement[0] + CameraMount.xOffset) < CameraMount.distThreshold)
                            {
                                runIntake();

                                increaseSlidesLengthInches(slidesLength * CameraMount.Scalar);
                            }
                            blockMovement = (displacement[0] + CameraMount.xOffset) * CameraMount.DriveScalar ;
                            break;

                    }
                }
                else {
                    blockMovement = 0;
                }





                // Some failsafes to prevent intaking another team's sample


            }
        }
    }

    private void reject() {
        reverseIntake();
        intakeMid();
        setSlidesLengthInches(20);
        if (stateTime.seconds() < 0.3) setState(visonState);
    }



    private double[] getDisplacement(boolean yellow, boolean team) {
        double [] teamSpecific = Constants.team == Constants.Team.BLUE ? processor.blueDisplacement : processor.redDisplacement;

        if (team == false) return processor.yellowDisplacement;
        // Even if looking for yellow, see if there is an alliance-specific sample closer
        if (yellow == false) return teamSpecific;
        if (((Math.abs(processor.yellowDisplacement[0] + CameraMount.xOffset)) < (Math.abs(teamSpecific[0] + CameraMount.xOffset)) && processor.yellowDisplacement[0] != 0 ) || teamSpecific[0] == 0) return processor.yellowDisplacement;
        else return teamSpecific;
    }

    public void runIntake(){
        intakeLeft.set(-1);
        intakeRight.set(1);
    }
    public void stopIntake(){
        intakeLeft.set(0);
        intakeRight.set(0);
    }
    public void reverseIntake(){
        intakeLeft.set(1);
        intakeRight.set(-1);
    }



    public SampleColor getIntakeColor(){
        colors = intakeSensor.getNormalizedColors();
        double distance = distanceSensor.getDistance(DistanceUnit.CM);
        Color.colorToHSV(colors.toColor(), hsvValues);

        BaseOpMode.addData("H", hsvValues[0]);
        BaseOpMode.addData("S", hsvValues[1]);
        BaseOpMode.addData("V", hsvValues[2]);

        BaseOpMode.addData("Distance", distance);

        //confirm there is a sample
        if (distance < 2.7){

            //within these
            if (hsvValues[0] > 50 && hsvValues[0] < 100) return SampleColor.YELLOW;
            else if (hsvValues[0] == 0 && hsvValues[1] == 1) return SampleColor.RED;
            else return  SampleColor.BLUE;


        }
        else return SampleColor.EMPTY;


    }
    public boolean checkForSample(){
        return getIntakeColor() != SampleColor.EMPTY;
    }



    public enum IntakeStates{
        HOMEDOWNNOTACTIVE, HOMEUP, HOMEACTIVE, EXTENDEDACTIVE, EXTENDEDUP, EXTENDEDREVERSED, HOMEREVERSED,
        TRANSFER, SEARCHNOYELLOW, SEARCHJUSTYELLOW, SEARCHBOTH, EXTENDEDNOTACTIVE, HOMEUPIN, HOMEUPOUT, NOTHING, REJECT;
    }
    public void setState(IntakeStates state){
        stateTime.reset();
        visionSampleReady = false;
        this.state = state;
        Constants.intakeReady = false;
    }
    public IntakeStates getState(){
        return state;
    }

    public enum SampleColor{
        YELLOW, NOTYELLOW, EMPTY, RED, BLUE
    }
    public void update(){
        if(!slidesResetting) {
            updateSlides();
        } else {
            resetSlides();
        }
            //
        work();
       // IntoTheDeepVisionProcessor.upperAreaDetectionLimit = (int) (inchesToPixels*(IMG_HEIGHT - 17 - getSlidesLengthInches()-lengthOfIntakeIN));
        //total exctension in inches is 17 inches

        //colors = colorSensor.getNormalizedColors();
        //Color.colorToHSV(colors.toColor(), hsvValues);
    }
    public void updateSensors(){

    }
}
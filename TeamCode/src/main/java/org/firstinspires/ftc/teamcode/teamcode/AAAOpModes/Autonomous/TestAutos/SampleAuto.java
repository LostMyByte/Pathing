/*
package org.firstinspires.ftc.teamcode.AAAOpModes.Autonomous.TestAutos;

import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.AAA_Paths.Path.BoardToInterimBlueGate;
import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.AAA_Paths.Path.CheckForTags;
import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.AAA_Paths.Path.HoldPark;
import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.AAA_Paths.Path.InterimToBoardBlueGate;
import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.AAA_Paths.Path.DropYellow;
import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.AAA_Paths.Path.Intake;
import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.AAA_Paths.Path.StartToSpike;
import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.AAA_Paths.Path.InterimToStackBlueGate;
import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.AAA_Paths.Path.DropWhite;
import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.AAA_Paths.Path.StackToInterimBlueGate;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.AAA_Paths;
import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.teamcode.KCP.Localization.TwoWheelOdometry;
import org.firstinspires.ftc.teamcode.teamcode.KCP.Movement;

import org.firstinspires.ftc.teamcode.Utilities.Vision.BlueProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "Blue 2 Cycle Auto")
public class BlueFrontGate2Cycle extends BaseOpMode {

    Movement drive;
    AAA_Paths.Path state;
    ElapsedTime timeWaste = new ElapsedTime();
    ElapsedTime totalTime;
    Intake intake;
    Scoring scoring;
    AprilTagProcessor aprilTags2;
    BlueProcessor visionProcessor = new BlueProcessor();
    WebcamName webcam1, webcam2;
    double rectPosition;
    boolean readyToDrive = false;
    double globalPower;
    //the randomization position that vision sees
    double randomPosition;
    public int searchForTag;
    private VisionPortal visionPortal1;
    private VisionPortal visionPortal2;
    AAA_Paths.Path nextState;
    int camera;
    int cycles = 0;
    int targetCycles;
    boolean notDone1 = true;
    boolean dropPurple = true;
    boolean notDone2 = true;
    boolean notDone3 = true;
    Constants pathingConstants;


    //TODO DELETE
    //Servos.V4BServoIntake v4b;

    double targetHeading = Math.PI/2;

    @Override
    public void externalInit() {

        camera = 2;
        pathingConstants = new Constants();
       // drive = new Movement(halfRobotSizeY, 79.5 + halfRobotSizeX, targetHeading);

        aprilTags2 = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .setLensIntrinsics(630.055, 630.055, 320.468, 266.21)
                .build();

        telemetry.update();
        webcam2 = hardwareMap.get(WebcamName.class, "Webcam 1");
        intake = new Intake();
        scoring = new Scoring();
        totalTime = new ElapsedTime();

        // Create the vision portal by using a builder
        int[] viewId = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);

        visionPortal2 = new VisionPortal.Builder()
                .setCamera(webcam2)
                .addProcessors(aprilTags2, visionProcessor)
                .setLiveViewContainerId(viewId[1])
                .build();

        //purple is webcam 2, white is webcam 1
        totalTime.reset();

        //TODO DELETE
        //v4b = new Servos.V4BServoIntake();
        //intake = new Intake();
        //scoring = new Scoring();
        //scoring.setState(Scoring.ScoreState.START_AUTO);
        //intake.setState(HOME);

        BoardToInterimBlueGate.compile();
        InterimToStackBlueGate.compile();
        StackToInterimBlueGate.compile();
        InterimToBoardBlueGate.compile();


        BaseOpMode.addData("X", TwoWheelOdometry.x());
        BaseOpMode.addData("Y", TwoWheelOdometry.y());
        BaseOpMode.addData("Heading", TwoWheelOdometry.heading());

        //camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
        //@Override
        //public void onOpened() {
        //camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

//this just sets up the camera
        //  camera.resumeViewport();

    }

    //@Override
    //public void onError(int errorCode) {
//When camera doesn't work nothing happens
    //}
    //});
    //FtcDashboard.getInstance().startCameraStream(camera, 30);

    //}

    @Override
    public void externalInitLoop() {
        rectPosition = visionProcessor.getRectPos();
        BaseOpMode.addData("pos", rectPosition);
        //BaseOpMode.addData("raw data, center of prop", pipeline.getCenter());
        //BaseOpMode.addData("equation for center", pipeline.getCenter() > IMG_WIDTH / 5 && pipeline.getCenter() < IMG_WIDTH - (IMG_WIDTH / 3));
        BaseOpMode.addData("img width", IMG_WIDTH);
        // aprilTags2.update();
        //aprilTags2.allTelemetry();
        randomPosition = rectPosition;
        setState(StartToSpike);
        scoring.setState(Scoring.ScoreState.POST_TRANSFER);
        intake.setState(HOME);

    }

    @Override
    public void externalStart() {
        timeWaste.reset();
        totalTime.reset();
        drive.setHeading(targetHeading);
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
//        if (totalTime.seconds() > 29) {
//            intake.setState(HOME);
//        }
        if (totalTime.seconds() > 29){
            pathingConstants.setEndAngle(drive.getHeading());
        }
        searchForTag = 3;
    }

    @Override
    public void externalStop() {

    }

    public void stateMachine() {
        switch (state) {
            case StartToSpike:
                startToSpike();
                break;
            case DropYellow:
                dropYellow();
                break;
            case HoldPark:
                holdPark();
                break;
            case BoardToInterimBlueGate:
                boardToInterim();
                break;
            case InterimToStackBlueGate:
                interimToStack();
                break;
            case StackToInterimBlueGate:
                stackToInterim();
                break;
            case InterimToBoardBlueGate:
                interimToBoard();
                break;
            case Intake:
                intake();
                break;
            case DropWhite:
                dropWhite();
                break;
            case CheckForTags:
                checkForTags();
                break;
            case StopState:
                stopState();
                break;
            case ShimmyShimmyAhShimmyAhShimmyAhDrankSwaLaLaLa:
                shimmyShimmyAhShimmyAhShimmyAhDrankSwaLaLaLa();
                break;
        }
    }

    public void startToSpike() {



        if (timeWaste.seconds() > 1 && notDone1) {
            targetHeading = targetHeading + Math.PI/2;
            notDone1 = false;
        }
        if (timeWaste.seconds() > 2 && timeWaste.seconds() < 3 && notDone2) {
            notDone2 = false;
            intake.setState(DOWN_CLOSE);
            intake.setIntakeMode(false);
        }
        if (randomPosition == 1) {
            drive.holdPosition(centerStackBlue - 7, fieldSizeY - 48 + halfRobotSizeYIntakeExtended, targetHeading);
        }
        if (randomPosition == 2) {
            drive.holdPosition(centerStackBlue, fieldSizeY - 59 + halfRobotSizeYIntakeExtended, targetHeading);
        }
        if (randomPosition == 3) {
            drive.holdPosition(centerStackBlue - 13, fieldSizeY - 70 + halfRobotSizeYIntakeExtended, targetHeading);
        }

        if (timeWaste.seconds() > 3) {
            if (totalTime.seconds() < 29 && dropPurple) {
                intake.setState(DOWN_OPEN);
                dropPurple = false;
            }
            if (timeWaste.seconds() > 3.5) {
                nextState = DropYellow;
                setState(CheckForTags);
            }
        }
    }

    public void dropYellow() {

        intake.setState(HOME);
        if (randomPosition == 1 && timeWaste.seconds() < 3) {
            drive.holdPosition(tag1x, backdropY-halfRobotSizeY-4.7,targetHeading);
        }
        if (randomPosition == 2 && timeWaste.seconds() < 3) {
            drive.holdPosition(tag2x, backdropY-halfRobotSizeY-4.7, targetHeading);
        }
        if (randomPosition == 3 && timeWaste.seconds() < 3) {
            drive.holdPosition(tag3x, backdropY-halfRobotSizeY-4.7, targetHeading);
        }

        if (timeWaste.seconds() < 2.2) {
            if(notDone3){
                scoring.setSlidesHeight(100);
                scoring.setState(PRE_SCORE);
                notDone3 = false;
            }
        } else if (timeWaste.seconds() > 3.5) {
            setState(BoardToInterimBlueGate);

        } else if (timeWaste.seconds() > 2.2 && notDone1){
            scoring.setState(SCORE_BOTH);
            notDone1 = false;
        }
    }


    public void boardToInterim() {
        scoring.setSlidesHeight(0);
        double power = .9;

        if (!drive.followPath(BoardToInterimBlueGate, power, targetHeading, .7, false)) {
            setState(InterimToStackBlueGate);
        }
    }

    public void interimToStack() {
        if(notDone1){
            globalPower = .8;}
        notDone1 = false;

        if(InterimToStackBlueGate.t > .65 && notDone3){
            globalPower = .3;
            intake.setStackHeight(4);
            intake.setState(DOWN_OPEN);
            intake.setIntakeMode(true);
            notDone3 = false;
        }

        if (!drive.followPath(InterimToStackBlueGate, globalPower, targetHeading, .9, true)) {
            setState(Intake);
        }
    }
    public void intake() {
        //Change this to make it go to stack height

        drive.holdPosition(gateStackBlue+1, halfRobotSizeYIntakeExtended, Math.PI);
        if (timeWaste.seconds() > 1.6){
            intake.setState(DOWN_CLOSE);
        }
        if (timeWaste.seconds() > 2) {
            setState(StackToInterimBlueGate);
        }
    }

    public void shimmyShimmyAhShimmyAhShimmyAhDrankSwaLaLaLa(){
        if (timeWaste.seconds() < 0.5) {
            drive.holdPosition(gateStackBlue, halfRobotSizeYIntakeExtended + 3, Math.PI);
        }
        if (timeWaste.seconds() > 0.5) {
            drive.holdPosition(gateStackBlue, halfRobotSizeYIntakeExtended + 2, Math.PI);
        }
        if (timeWaste.seconds() > 1) {
            setState(StackToInterimBlueGate);
        }
    }

    public void stackToInterim() {
        if (StackToInterimBlueGate.t < .3 && notDone1) {
            notDone1 = false;
            intake.setState(HOME);
            if(StackToInterimBlueGate.t > .2){
                scoring.bump();
            }
        }
        if (StackToInterimBlueGate.t > .3 && notDone3){
            scoring.setState(TRANSFER);
            intake.setState(org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeState.TRANSFER);
            notDone3 = false;
        }

        if (!drive.followPath(StackToInterimBlueGate, .9, targetHeading, .9, false)) {
            setState(InterimToBoardBlueGate);
        }
    }

    public void interimToBoard() {
        double power = .9;

        if(InterimToBoardBlueGate.t > .75){
            power = .3;
        }

        if (!drive.followPath(InterimToBoardBlueGate, power, targetHeading, .7, true)) {
            nextState = DropWhite;
            setState(CheckForTags);
        }
    }


    public void dropWhite() {
        if (timeWaste.seconds() < 2) {
            drive.holdPosition(tag3x, Constants.backdropY - 13, Math.PI);
        }
        if (timeWaste.seconds() < 2 && notDone3 && !scoring.initiateTransfer) {
            notDone3 = false;
            scoring.setState(PRE_SCORE);
            scoring.setSlidesHeight(400);
        }
        if (timeWaste.seconds() > 2.7 && notDone1) {
            scoring.setState(SCORE_BOTH);
            notDone1 = false;
            cycles += 1;
        }
        if(timeWaste.seconds() > 2.7){
            drive.holdPosition(9,fieldSizeY-20,Math.PI);
        }
        if(timeWaste.seconds() > 3){
            if (cycles < 2) {
                setState(BoardToInterimBlueGate);
            } else {
                setState(HoldPark);
            }
        }

    }

    public void holdPark() {
        scoring.setSlidesHeight(-200);
        scoring.setState(Scoring.ScoreState.HOME);
        intake.setState(HOME);
        drive.holdPosition(9,fieldSizeY-20,Math.PI);
        readyToDrive = true;
    }

    public void checkForTags() {
        boolean tagsDone = false;
        boolean timedOut = false;
//        if (aprilTags2.getDetections().size() != 0){
//            searchForTag = aprilTags2.getDetections().get(0).id;
//        }

        BaseOpMode.addData("apriltags", getClosestByID(searchForTag));
        BaseOpMode.addData("aprilTagCount", aprilTags2.getDetections().size());
        BaseOpMode.addData("time", timeWaste.seconds());
        AprilTagDetection correctAT = getClosestByID(searchForTag);



        if (correctAT != null && searchForTag == correctAT.id) {
            BaseOpMode.addData("foundTag","");

//            //if this works properly will return id of index 0 in list of detections this loop
//            if (aprilTags.getDetections().size() != 0) {
//                searchForTag = aprilTags.getDetections().get(0).id;
//            }
            drive.update(correctAT, correctAT.ftcPose.x, correctAT.ftcPose.y, correctAT.ftcPose.yaw, 2);
            BaseOpMode.addData("tagX", correctAT.ftcPose.x);
            BaseOpMode.addData("tagY", correctAT.ftcPose.y);
            BaseOpMode.addData("tagYaw", correctAT.ftcPose.yaw);
        }
        if (timeWaste.seconds() > 1) {
            tagsDone = true;
            BaseOpMode.addData("CATASTROPHIC: TAG DETECTION TIMED OUT", "");
        }

        BaseOpMode.addData("Tag Detected",tagsDone);
        if(tagsDone) {
            setState(nextState);
        }
    }
    public void stopState(){
        drive.stopDrive();
    }

    public void setState(AAA_Paths.Path state){
        //visionPortal.setActiveCamera(camera == 1 ? webcam1:webcam2);
        this.state = state;
        timeWaste.reset();
        readyToDrive = false;
        notDone1 = true;
        notDone2 = true;
        notDone3 = true;
    }

    public AprilTagDetection getClosestByID(int id){
        List<AprilTagDetection> currentDetections = aprilTags2.getDetections();
        if(currentDetections == null || currentDetections.isEmpty()){
            return null;
        }

        for(AprilTagDetection detection : currentDetections){
            if(detection.id == id){
                return detection;
            }
//
        }
        return null;
    }

    public int apriltagsAnyDetections(){
        return aprilTags2.getDetections().get(0).id;
    }
}
*/

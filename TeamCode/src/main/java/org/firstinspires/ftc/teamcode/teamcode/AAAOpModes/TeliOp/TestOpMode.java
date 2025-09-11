package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.TeliOp;

import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;

import org.firstinspires.ftc.teamcode.teamcode.KCP.Localization.AprilTagOdometrySource;
import org.firstinspires.ftc.teamcode.teamcode.KCP.Localization.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.teamcode.KCP.Movement;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.DiffyBoxtube;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.MainIntake;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Scoring;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Servos;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Dash.DashPositions;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.HardwareDevices.Motor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name="Intake Testing", group="Iterative Opmode")
public class TestOpMode extends BaseOpMode {


    //private Movement drive;

    double ax, ay;

    private AprilTagOdometrySource aprilTags;

    ElapsedTime timeWaste = new ElapsedTime();
    WebcamName webcam;
    Scoring scoring;
    //Servos.BottomSweeper sweeper;
    //Servos.DifferentialLeft differentialLeft;
    //Servos.DifferentialRight differentialRight;
    Servos.DepositorClaw claw;

    VisionPortal vision;

    GoBildaPinpointDriver odo;
    CRServo intakeLeft;
    CRServo intakeRight;
    Servos.DifferentialLeft intakeArmLeft;
    Servos.DifferentialRight intakeArmRight;
    NormalizedColorSensor colorSensor;
    final float[] hsvValues = new float[3];

    MainIntake intake;
    Movement movement;
    TouchSensor limitSwitch;
    DiffyBoxtube boxtube;
    Motor motor;

    @Override
    public void externalInit() {
        //drive = new Movement(0, 0, 0);
        //claw = new Servos.DepositorClaw();
        //ax = drive.getX();
        //ay = drive.getY();
        //aprilTags = new AprilTagOdometrySource(24,24, 0, "blue");
/*

        aprilTags = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getIntoTheDeepTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .setLensIntrinsics(fx, fy, cx, cy)
                .build();
*/
        //telemetry.update();
        //webcam = hardwareMap.get(WebcamName.class, Hardware.tagCam);
        /*
        vision = new VisionPortal.Builder()
                .setCamera(webcam)
                .addProcessor(drive.getProcessor())
                .setCameraResolution(new Size(1280, 800))
                .build();


        FtcDashboard.getInstance().startCameraStream(vision, 10);


        */

        //intake = new MainIntake(hardware);
        //intake.setState(MainIntake.IntakeStates.HOMEUP);
        //movement = new Movement(0,0,0);
        //sweeper = new Servos.BottomSweeper();
        //intakeArmLeft = new Servos.DifferentialLeft();
        //intakeArmRight = new Servos.DifferentialRight();
        //intakeLeft = new CRServo(hardwareMap, Hardware.intakeLeft);
        //intakeRight = new CRServo(hardwareMap, Hardware.intakeRight);
        //scoring = new Scoring(hardwareMap);
        //intake = new MainIntake(hardwareMap);
        //limitSwitch = hardwareMap.get(TouchSensor.class, "magnetSensor");
        boxtube = new DiffyBoxtube(hardwareMap);

        //motor = new Motor("motorR", false, true);
    }

    @Override
    public void externalLoop() {

        BaseOpMode.addData("angle", boxtube.getCurrentAngle());
        BaseOpMode.addData("extension", boxtube.getExtensionM());

        boxtube.setTargetPosition(DashPositions.servoTest, DashPositions.dashboardPositionSlides);

        //motor.setPower(1);
        //BaseOpMode.addData("motorRPM", motor.encoder.getVelocity());
        //claw.setPosition(DashPositions.servoTest);
        //multTelemetry.addData("X", drive.getX());
        //multTelemetry.addData("Y", drive.getY());
//        if (driver1.dpad_up.isPressed()) {
//            scoring.setSlidesHeight(2000);
//        } else if (driver1.dpad_down.isPressed()){
//            scoring.setSlidesHeight(100);
//        }
        //BaseOpMode.addData("left", DashPositions.left);
        //scoring.updateSlides(DashPositions.dashboardPositionSlides);
        //scoring.setDepositorAngleInterpolated(DashPositions.servoTest);
        //scoring.differentialRaw(DashPositions.right, DashPositions.left);


        //BaseOpMode.addData("color",intake.getIntakeColor());
        //BaseOpMode.addData("magnet", limitSwitch.isPressed());
/*





        multTelemetry.addData("X", drive.getX());
        multTelemetry.addData("Y", drive.getY());
        if (gamepad1.dpad_up) {
            ay += 5;
            sleep(100);
        }
        else if (gamepad1.dpad_down) {
            ay -= 5;
            sleep(100);
        }
        else if (gamepad1.dpad_left) {
            ax -= 5;
            sleep(100);
        }
        else if (gamepad1.dpad_right) {
            ax += 5;
            sleep(100);
        }
        //drive.holdPosition(ax,ay, 0, 1);
*/




    }


    public void checkForTags() {
        boolean tagsDone = false;
        boolean timedOut = false;
        timeWaste.reset();

        BaseOpMode.addData("apriltags id", aprilTags.getId(0));
        //BaseOpMode.addData("aprilTagCount", aprilTags.getDetectionsSize());
        BaseOpMode.addData("time", timeWaste.seconds());
        AprilTagDetection correctAT = aprilTags.getClosestByID(aprilTags.getId(0));



        if (correctAT != null) {
            BaseOpMode.addData("foundTag","");

//            if this works properly will return id of index 0 in list of detections this loop


            //drive.update(); //, correctAT.ftcPose.x, correctAT.ftcPose.y, correctAT.ftcPose.yaw);
            BaseOpMode.addData("tagX",correctAT.ftcPose.x);
            BaseOpMode.addData("tagY",correctAT.ftcPose.y);
            BaseOpMode.addData("tagYaw",correctAT.ftcPose.yaw);
            BaseOpMode.addData("tag range", correctAT.ftcPose.range);

            tagsDone = true;
        }
//        if (timeWaste.seconds() > 1) {
//            tagsDone = true;
//            timedOut = true;
//            BaseOpMode.addData("CATASTROPHIC: TAG DETECTION TIMED OUT", "");
//        }

        BaseOpMode.addData("Tag Detected",tagsDone);
        BaseOpMode.addData("Tags Timed Out",timedOut);

        }

    }

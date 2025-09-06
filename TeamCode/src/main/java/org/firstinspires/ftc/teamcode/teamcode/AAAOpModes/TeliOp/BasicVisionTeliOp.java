package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.TeliOp;


import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Constants.Team.BLUE;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;

import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Constants;

import org.firstinspires.ftc.teamcode.teamcode.Utilities.Vision.BlockDetection;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Vision.IntoTheDeepVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

//@Disabled

@TeleOp(name="AAAA vision", group="Iterative Opmode")
public class BasicVisionTeliOp extends BaseOpMode {
    private IntoTheDeepVisionProcessor visionProcessor = new IntoTheDeepVisionProcessor();


    private ElapsedTime timer;
    double lastTime;
    private VisionPortal visionPortal;
    private WebcamName webcam1;
    //Movement drive;
   // AprilTagLibrary aprilTagLibrary ;
    BlockDetection limelight;
    //limelight stuff is all in this class
    Size cameraResolution = new Size(1280, 720);

    private double holdingA = 0;

    FtcDashboard dash;
    //DiffyClaw claw;
    @Override
    public void externalInit() {

        timer = new ElapsedTime();
        //This is the code that plays when you press INIT
//aprilTagLibrary = new AprilTagLibrary.Builder().addTag(5,"tag5",6.5,DistanceUnit.INCH).build();

        dash = FtcDashboard.getInstance();
        //claw = new DiffyClaw();

     //   drive = new Movement(24, 24, 0);
       Constants.team = BLUE;
        //webcam1 = hardware.get(WebcamName.class, Hardware.tagCam);
         webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        visionPortal = new VisionPortal.Builder()
                //setup for using webcam, there is a different way to set up a phone camera
                .setCamera(webcam1)
                //use addProcessor() for only adding one processor
                .addProcessors(visionProcessor)
               // .addProcessors(drive.getProcessor())
                //sets the camera resolution to the size we set up earlier
                //Currently 1280x720 because that size works for both a global shutter camera and a logitech camera
                .setCameraResolution(cameraResolution)
                .build();
        //this is what allows ftc dashboard to work
       dash.startCameraStream(visionPortal, 0);
      //this line is used for this type of teliOp set up but if your teliOp or auto extends
        //something that isn't baseOpMode, you won't need this

        //dash.startCameraStream(visionPortal, 20);

        waitForStart();
    }

    @Override
    public void externalInitLoop(){
        //Code that keeps looping in init until you press start
    }

    @Override
    public void externalStart(){
        //code that plays once when you press start
    }

    @Override
    public void externalLoop() {
        //Code that loops throughout the opmode
        // claw.moveToPickUpSample(limelight.angleBlue, limelight.centerBlue);


        //drive.holdPosition(24,24, drive.getHeading());


        //if(getClosestByID(5)!=null){
        //    multTelemetry.addData("detection x distance", getClosestByID(5).ftcPose.x);}
        //else {multTelemetry.addData("nothing","detected");}
        //Apriltags!

       /* double t = timer.time();
        if ((holdingA == 0 && (t-lastTime) > 1) || (holdingA == 1 && (t-lastTime) < 1)) {
            boolean result = drive.holdPosition(24, 36, 0, 0.5);
            if (result) {
                holdingA = 1;
                lastTime = (t-lastTime > 1) ? t : lastTime;
            }

        }
        else if ((holdingA == 1 && (t-lastTime) > 1) || (holdingA == 2 && (t-lastTime) < 1)) {
            boolean result = drive.holdPosition(36, 36, Math.PI/2, 0.5);
            if (result) {

                holdingA = 2;
                lastTime = (t-lastTime > 1) ? t : lastTime;
            }
        }
        else if ((holdingA == 2 && (t-lastTime) > 1) || (holdingA == 3 && (t-lastTime) < 1)) {
            boolean result = drive.holdPosition(36, 24, Math.PI, 0.5);
            if (result) {

                holdingA = 3;
                lastTime = (t-lastTime > 1) ? t : lastTime;
            }
        }
        else if ((holdingA == 3 && (t-lastTime) > 1) || (holdingA == 0 && (t-lastTime) < 1)) {
            boolean result = drive.holdPosition(24, 24, 0, 0.5);
            if (result) {

                holdingA = 0;
                lastTime = (t-lastTime > 1) ? t : lastTime;
            }
        }

        addData("Drive X", drive.getX());
        addData("Drive Y", drive.getY());
*/
        multTelemetry.addData("top of detection area", IntoTheDeepVisionProcessor.upperAreaDetectionLimit);
        multTelemetry.addData("block angle", visionProcessor.getAngle());
        multTelemetry.addData("block angle yellow", visionProcessor.getAngleYellow());

        // Handle displacement without yellow
        double[] displacement = visionProcessor.getDisplacement(false);
        if (displacement != null){ //&& displacement.length >= 2) {
            multTelemetry.addData("distance x", displacement[0]);
            multTelemetry.addData("distance y", displacement[1]);
        } else {
            multTelemetry.addData("distance x", "No data");
            multTelemetry.addData("distance y", "No data");
        }

        // Handle displacement with yellow
        double[] displacementYellow = visionProcessor.getDisplacement(true);
        if (displacementYellow != null && displacementYellow.length >= 2) {
            multTelemetry.addData("distance x yellow", displacementYellow[0]);
            multTelemetry.addData("distance y yellow", displacementYellow[1]);
        } else {
            multTelemetry.addData("distance x yellow", "No data");
            multTelemetry.addData("distance y yellow", "No data");
        }

        multTelemetry.addData("target detected", visionProcessor.targetDetected);

    }

    }

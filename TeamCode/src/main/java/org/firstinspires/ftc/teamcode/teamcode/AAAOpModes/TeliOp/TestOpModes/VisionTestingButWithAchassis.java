package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.TeliOp.TestOpModes;

import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Hardware.cameraResolution;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Vision.BallChaser;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp (name = "AAA visionTest")
public class VisionTestingButWithAchassis extends BaseOpMode {
    private FtcDashboard dash;
    Drivetrain drive;

    WebcamName webcam1;
    BallChaser ballChaser;

    VisionPortal visionPortal;
    ElapsedTime timewaste;
    public Boolean isTargeting = false;



    @Override
    public void externalInit() {
        ballChaser = new BallChaser();
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        visionPortal = new VisionPortal.Builder()
                //setup for using webcam, there is a different way to set up a phone camera
                .setCamera(webcam1)
                //use addProcessor() for only adding one processor
                .addProcessor(ballChaser)
                // .addProcessors(drive.getProcessor())
                //sets the camera resolution to the size we set up earlier
                //Currently 1280x720 because that size works for both a global shutter camera and a logitech camera
                .setCameraResolution(cameraResolution)
                .build();

        drive = new Drivetrain(hardwareMap, 0);
        timewaste = new ElapsedTime();
        FtcDashboard.getInstance().startCameraStream(visionPortal, 0);
        //this is what allows ftc dashboard to work
        //dash.startCameraStream(visionPortal, 0);
        telemetry = FtcDashboard.getInstance().getTelemetry();
        waitForStart();
        waitForStart();
    }

    @Override
    public void externalLoop() {
        timewaste.reset();
        BaseOpMode.addData("is targeting", isTargeting);
        BaseOpMode.addData("time", timewaste);
        if(driver1.triangle.isTapped()){
            isTargeting=!isTargeting;
        }
        if(!driver1.circle.isTapped()&&isTargeting){
     drive.ballFollow(138.0);
    }
       if (driver1.square.isTapped()&&!driver1.circle.isTapped()&&isTargeting){
            while (timewaste.seconds() < 5) {
                drive.ballFollow(250.0);
            }
            timewaste.reset();
            while (timewaste.milliseconds()<500){
            drive.tankDrive(1,0,0.5);
            isTargeting = false;
        }}
    }

}

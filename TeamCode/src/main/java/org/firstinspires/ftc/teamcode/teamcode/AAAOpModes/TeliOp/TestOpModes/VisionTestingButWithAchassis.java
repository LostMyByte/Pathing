package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.TeliOp.TestOpModes;

import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Hardware.cameraResolution;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Vision.BallDetector;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp (name = "AAA visionTest")
public class VisionTestingButWithAchassis extends BaseOpMode {
    private FtcDashboard dash;
    Drivetrain drive;

    WebcamName webcam1;
    BallDetector ballDetector;

    VisionPortal visionPortal;



    @Override
    public void externalInit() {

        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        visionPortal = new VisionPortal.Builder()
                //setup for using webcam, there is a different way to set up a phone camera
                .setCamera(webcam1)
                //use addProcessor() for only adding one processor
                .addProcessors(ballDetector)
                // .addProcessors(drive.getProcessor())
                //sets the camera resolution to the size we set up earlier
                //Currently 1280x720 because that size works for both a global shutter camera and a logitech camera
                .setCameraResolution(cameraResolution)
                .build();
        //this is what allows ftc dashboard to work
        dash.startCameraStream(visionPortal, 0);

        drive = new Drivetrain(hardwareMap,0);
        dash = FtcDashboard.getInstance();
        telemetry = dash.getTelemetry();
        waitForStart();
    }

    @Override
    public void externalLoop() {
        if(!driver1.circle.isTapped()){
     drive.ballFollow();
    }}
}

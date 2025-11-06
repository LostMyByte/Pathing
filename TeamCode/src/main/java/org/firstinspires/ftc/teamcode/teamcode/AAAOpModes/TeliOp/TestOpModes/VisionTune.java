// Primary Author: Caroline Oringer
package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.TeliOp.TestOpModes;

import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Hardware.cameraResolution;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Vision.BallChaser;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
@TeleOp (name = "visionTune")
public class VisionTune extends BaseOpMode {

    private FtcDashboard dash;
    WebcamName webcam1;
    BallChaser ballDetector;

    VisionPortal visionPortal;



    @Override
    public void externalInit() {

        ballDetector = new BallChaser();
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        visionPortal = new VisionPortal.Builder()
                //setup for using webcam, there is a different way to set up a phone camera
                .setCamera(webcam1)
                //use addProcessor() for only adding one processor
                .addProcessor(ballDetector)
                // .addProcessors(drive.getProcessor())
                //sets the camera resolution to the size we set up earlier
                //Currently 1280x720 because that size works for both a global shutter camera and a logitech camera
                .setCameraResolution(cameraResolution)
                .build();



        FtcDashboard.getInstance().startCameraStream(visionPortal, 0);
        //this is what allows ftc dashboard to work
        //dash.startCameraStream(visionPortal, 0);
        telemetry = FtcDashboard.getInstance().getTelemetry();
        waitForStart();
    }

    @Override
    public void externalLoop() {
      //  multTelemetry.addData("Color RBG to HSV", ballDetector.getCenterpixel(false));
      //  multTelemetry.addData("Color BGR to HSV", ballDetector.getCenterpixel(true));
        multTelemetry.update();
    }
}

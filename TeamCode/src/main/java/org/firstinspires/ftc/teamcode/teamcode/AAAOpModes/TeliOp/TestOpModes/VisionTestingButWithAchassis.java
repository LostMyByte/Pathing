// Primary Author: Caroline Oringer
package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.TeliOp.TestOpModes;

import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Hardware.cameraResolution;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains.Movement;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Vision.BallDetector;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
@TeleOp (name = "AAA visionTest")
public class VisionTestingButWithAchassis extends BaseOpMode {
    private FtcDashboard dash;
    Movement drive;

    WebcamName webcam1;
    BallDetector ballDetector;

    VisionPortal visionPortal;



    @Override
    public void externalInit() {
        ballDetector = new BallDetector();
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

        //drive = new Drivetrain(hardwareMap, 0);

        FtcDashboard.getInstance().startCameraStream(visionPortal, 0);
        //this is what allows ftc dashboard to work
        //dash.startCameraStream(visionPortal, 0);
        telemetry = FtcDashboard.getInstance().getTelemetry();
        waitForStart();
        waitForStart();
    }

    @Override
    public void externalLoop() {
        if(!driver1.circle.isTapped()){
     //drive.ballFollow();
    }
    }

}

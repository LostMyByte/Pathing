package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Testing;

import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Hardware.cameraResolution;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Controllers.BallFollower;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains.TankDriveTrain;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Vision.BallDetector;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name = "KieranVisionTesting")
public class KieranVisionTest extends BaseOpMode {

    TankDriveTrain drive;
    BallFollower controller;
    WebcamName webcam;
    BallDetector ballDetector;

    VisionPortal visionPortal;


    @Override
    public void externalInit() {
        ballDetector = new BallDetector();
        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        visionPortal = new VisionPortal.Builder()
                //setup for using webcam, there is a different way to set up a phone camera
                .setCamera(webcam)
                //use addProcessor() for only adding one processor
                .addProcessor(ballDetector)
                // .addProcessors(drive.getProcessor())
                //sets the camera resolution to the size we set up earlier
                //Currently 1280x720 because that size works for both a global shutter camera and a logitech camera
                .setCameraResolution(cameraResolution)
                .build();

        FtcDashboard.getInstance().startCameraStream(visionPortal, 0);

        drive = new TankDriveTrain();
        controller = new BallFollower(BallFollower.BallColor.PURPLE, 150);
    }

    @Override
    public void externalLoop() {
        drive.followController(controller);
    }
}

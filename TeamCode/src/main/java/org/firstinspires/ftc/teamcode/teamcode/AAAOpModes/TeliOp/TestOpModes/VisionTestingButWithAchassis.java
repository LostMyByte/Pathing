// Primary Author: Caroline Oringer
package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.TeliOp.TestOpModes;

import static org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Hardware.cameraResolution;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Vision.BallChaser;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
@TeleOp (name = "AAA visionTest")
public class VisionTestingButWithAchassis extends BaseOpMode {
    private FtcDashboard dash;
    Drivetrain drive;

    WebcamName webcam1;
    BallChaser ballDetector;

    VisionPortal visionPortal;
    ElapsedTime timewaste;
    public Boolean isTargeting = false;
    Boolean isIntakeing = false;




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

        drive = new Drivetrain(0);
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
        BaseOpMode.addData("is targeting", isTargeting);
        BaseOpMode.addData("time", timewaste);
        if(driver1.triangle.isTapped()){
            isTargeting=!isTargeting;
        }
        if(!driver1.circle.isTapped()&&isTargeting&&!isIntakeing){
            drive.ballFollow(138, false);
        }
        if ((driver1.square.isTapped()&&!driver1.circle.isTapped()&&isTargeting)||isIntakeing) {
            if (isIntakeing == false){timewaste.reset();}
            isIntakeing = true;
            if (timewaste.seconds() < 15) {
                drive.ballFollow(190, false);
                BaseOpMode.addData("time 2", timewaste);

            }else {
                isIntakeing = false;
                timewaste.reset();
            }
        }
    }

}

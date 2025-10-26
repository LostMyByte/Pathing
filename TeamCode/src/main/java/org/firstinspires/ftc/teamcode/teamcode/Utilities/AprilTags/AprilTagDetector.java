// Primary Author: Caroline Oringer, Past Team Members
package org.firstinspires.ftc.teamcode.teamcode.Utilities.AprilTags;



import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

public class AprilTagDetector {

    public static int exposure = 5;

    private FtcDashboard dash;

    private OpenCvWebcam webcam;
    ExposureControl exposureControl;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C270 webcam at 640x480
    // Taken from FIRST approx calibration, may be different for each camera
    // Also, what res we're running at I'm still not entirely sure
    double fx = 686.658;
    double fy = 616.658;
    double cx = 212.598;
    double cy = 231.521;

    // UNITS ARE METERS - This is tuned to not actually be the tag size idk why
    public static double tagsize = 0.173;

    int numFramesWithoutDetection = 0;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

    int PARK_ZONE = 0; // Tag ID 18 from the 36h11 family

    AprilTagDetection tagOfInterest = null;
    ArrayList<AprilTagDetection> currentDetections;

    public AprilTagDetector(String webcamName){
        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(BaseOpMode.getHardwareMap().get(WebcamName.class, webcamName));
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        webcam.setPipeline(aprilTagDetectionPipeline);
        //webcam.openCameraDevice();

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()

        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(1280,800, OpenCvCameraRotation.UPSIDE_DOWN);
                exposureControl = webcam.getExposureControl();
                exposureControl.setMode(ExposureControl.Mode.Auto);
                exposureControl.setExposure(exposure, TimeUnit.MILLISECONDS);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        dash = FtcDashboard.getInstance();
        dash.startCameraStream(webcam, 20);

        webcam.setPipeline(aprilTagDetectionPipeline);


        //webcam.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);


    }

    public void update(){

        currentDetections = aprilTagDetectionPipeline.getDetectionsUpdate();

        if(currentDetections != null)
        {
            // If we don't see any tags
            if(currentDetections.size() == 0)
            {
                numFramesWithoutDetection++;

                // If we haven't seen a tag for a few frames, lower the decimation
                // so we can hopefully pick one up if we're e.g. far back
                if(numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION)
                {
                    aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                }
            }
            // We do see tags!
            else
            {
                numFramesWithoutDetection = 0;

                // If the target is within 1 meter, turn on high decimation to
                // increase the frame rate
                if(currentDetections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS)
                {
                    aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                }

                for(AprilTagDetection detection : currentDetections)
                {
                    BaseOpMode.addLine(String.format("\nDetected tag ID=%d", detection.id));
                    BaseOpMode.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
                    BaseOpMode.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
                    BaseOpMode.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
                    Orientation orientation = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
                    BaseOpMode.addLine(String.format("Rotation Yaw: %.2f degrees", orientation.firstAngle));
                    BaseOpMode.addLine(String.format("Rotation Pitch: %.2f degrees", orientation.secondAngle));
                    BaseOpMode.addLine(String.format("Rotation Roll: %.2f degrees", orientation.thirdAngle));
                }
            }
        }
    }


    public AprilTagDetection getSpecificTagID(int id){
        for(AprilTagDetection detection : currentDetections){
            if(detection.id == id){
                return detection;
            }
        }
        //if your tag is not detected returns null
        return null;
    }

    public void closePipeline(){
        webcam.closeCameraDevice();
    }

    public ArrayList<AprilTagDetection> getDetections(){
        return currentDetections;
    }
}

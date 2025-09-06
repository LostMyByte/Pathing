package org.firstinspires.ftc.teamcode.teamcode.Utilities.AprilTags;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTagDetectorNew {

    private AprilTagProcessor aprilTag;
    public VisionPortal visionPortal;
    private List<AprilTagDetection> currentDetections;

    public AprilTagDetectorNew(String webcamName){
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();


        //Library for FGC
        AprilTagLibrary.Builder libraryBuilder = new AprilTagLibrary.Builder();
        libraryBuilder.addTag(2, "BlueBackdropMiddle", 0.05, DistanceUnit.METER);
        libraryBuilder.addTag(3, "BlueBackdropRight", 0.05, DistanceUnit.METER);
        libraryBuilder.addTag(1, "BlueBackdropLeft", 0.05, DistanceUnit.METER);
        libraryBuilder.addTag(5, "RedBackdropMiddle", 0.05, DistanceUnit.METER);
        libraryBuilder.addTag(6, "RedBackdropRight", 0.05, DistanceUnit.METER);
        libraryBuilder.addTag(4, "RedBackdropLeft", 0.05, DistanceUnit.METER);
        libraryBuilder.addTag(9, "Blue Stack Small", 0.05, DistanceUnit.METER);
        libraryBuilder.addTag(8, "Red Stack Small", 0.05, DistanceUnit.METER);
        libraryBuilder.addTag(10, "Blue Stack Big", 0.127, DistanceUnit.METER);
        libraryBuilder.addTag(7, "Red Stack Big", 0.127, DistanceUnit.METER);


        AprilTagLibrary centerStage = libraryBuilder.build();

        aprilTag = new AprilTagProcessor.Builder()
                //.setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setTagLibrary(centerStage)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //for camera 1:
                //.setLensIntrinsics(306.799, 309.799, 199.305, 170.954)
                //for camera 2:
                .setLensIntrinsics(630.055, 630.055, 320.468, 266.21)
                //before I changed it:
                //.setLensIntrinsics(822.317, 822.317, 319.495, 242.502)

                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();



        builder.setCamera(BaseOpMode.getHardwareMap().get(WebcamName.class, webcamName));


        // Set and enable the processor.
        builder.addProcessor(aprilTag);


        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();
    }

    public void update(){
        currentDetections = aprilTag.getDetections();
    }

    public List<AprilTagDetection> getDetections(){
        return currentDetections;
    }

    public void telemetryAprilTags() {

        BaseOpMode.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                BaseOpMode.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                BaseOpMode.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                BaseOpMode.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                BaseOpMode.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                BaseOpMode.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                BaseOpMode.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        BaseOpMode.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        BaseOpMode.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        BaseOpMode.addLine("RBE = Range, Bearing & Elevation");

    }
    public int getNumDetections(){return currentDetections.size();}

    public void close(){
        visionPortal.close();
    }

}

// Primary Author: Caroline Oringer, Past Team Members
package org.firstinspires.ftc.teamcode.teamcode.Utilities.AprilTags;


import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;

public class AprilTagsSorter extends Subsystem {
    public AprilTagDetectorNew aprilTag;


    //ADD TAGS TO LIBRARY


    public AprilTagsSorter(String mapName){
        aprilTag = new AprilTagDetectorNew(mapName);
    }

    @Override
    public void update(){
        aprilTag.update();
    }
    public void update(AprilTagDetection tagNum, double x, double y, double yaw, double cameraNumber){}

    @Override
    public void updateSensors() {

    }

    public void allTelemetry(){
        aprilTag.telemetryAprilTags();
    }

    public ArrayList<AprilTagDetection> getSpecificTagID(int id){
        if(aprilTag.getDetections() == null || aprilTag.getDetections().isEmpty()){
            return null;
        }
        ArrayList<AprilTagDetection> retVal = new ArrayList<>();
        for(AprilTagDetection detection : aprilTag.getDetections()){
            if(detection.id == id){
                retVal.add(detection);
            }
        }
        //if your tag is not detected returns null
        return retVal;
    }

    public AprilTagDetection getClosestByID(int id){
        ArrayList<AprilTagDetection> list = getSpecificTagID(id);
        if(list == null || list.isEmpty()){
            return null;
        }
        AprilTagDetection retVal = list.get(0);
        for(AprilTagDetection detection : list){
//            BaseOpMode.addData("detection", detection);
            if(detection.ftcPose.range > retVal.ftcPose.range){
                retVal = detection;
            }
        }
        return retVal;
    }
    public int getNumDetections(){return aprilTag.getNumDetections();}

}

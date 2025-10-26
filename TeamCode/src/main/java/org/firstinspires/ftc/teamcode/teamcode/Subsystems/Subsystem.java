// Primary Author: Mixed, Past Team Members
package org.firstinspires.ftc.teamcode.teamcode.Subsystems;

import java.util.ArrayList;

public abstract class Subsystem {

    public static ArrayList<Subsystem> subsystems = new ArrayList<>();

    public Subsystem(){
        subsystems.add(this);
    }

    public abstract void update();
//    public abstract void update(AprilTagDetection tagNum, double x, double y, double heading, double cameraNumber);

    public abstract void updateSensors();

    public static void pingSensors(){
        for(Subsystem s : subsystems){
            s.updateSensors();
        }
    }

    public static void updateSubsystems(){
        for(Subsystem s : subsystems){
            s.update();
        }
    }

    public static void resetSubsystemList(){
        subsystems.clear();
    }

}

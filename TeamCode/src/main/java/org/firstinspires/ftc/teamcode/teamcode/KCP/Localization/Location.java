package org.firstinspires.ftc.teamcode.teamcode.KCP.Localization;


import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public abstract class Location extends Subsystem {

    protected static double[] location;
    protected static double heading;

    private final ElapsedTime runtime;

    protected final double[] velocity = new double[3];

    public Location(double x, double y){
        super();
        runtime = new ElapsedTime();
        runtime.reset();
        location = new double[]{x, y};
    }

    public static double x(){
        return location[0];
    }

    public static double y(){
        return location[1];
    }

    public static void shiftLocation(double x, double y){
        location[0] -= x;
        location[1] -= y;
    }

    public static void setPosition(double x, double y){
        location[0] = x;
        location[1] = y;
    }

    public static double heading(){

        double newHeading = heading;
        while (newHeading > 2 * Math.PI) {
            newHeading -= 2 * Math.PI;
        }

        while (newHeading < 0) {
            newHeading += 2 * Math.PI;
        }
        return heading;
    }
    public static double headingUnwrapped(){
        return heading;
    }

    public abstract void setCurrentHeading(double heading);
    public abstract void localize();


    /*public void update(AprilTagDetection tagNum) {
        BaseOpMode.addData("updated","");
        if(tagNum != null){
            BaseOpMode.addData("april localizing attempt","");
            aprilLocalize(tagNum);
        }else {
            localize();
        }
        updateVelocity();
    } */
    @Override
    public void update(){
        localize();
        //updateVelocity();
    }

    @Override
    public void updateSensors() {

    }

    private int velocityRingBufferCount = 0;
    private final double[][] velocityRingBuffer = new double[][] {{1,0, 0, 999999999},{2,0, 0, 999999999},{3,0, 0, 999999999},{4,0, 0, 999999999},{5,0, 0, 999999999},{6,0, 0, 999999999},{7,0, 0, 999999999},{8,0, 0, 999999999},{9,0, 0, 999999999},{10,0, 0, 999999999} };
    public void updateVelocity(){

        double time = runtime.milliseconds() / 1000;

        int current = velocityRingBufferCount % 5;
        int last = (velocityRingBufferCount+1) % 5;

        velocityRingBuffer[current][0] = Location.x();
        velocityRingBuffer[current][1] = Location.y();
        velocityRingBuffer[current][2] = Location.heading();
        velocityRingBuffer[current][3] = time;

        double dX = Location.x()-velocityRingBuffer[last][0];
        double dY = Location.y()-velocityRingBuffer[last][1];
        double dH = Location.heading()-velocityRingBuffer[last][2];
        double dT = time-velocityRingBuffer[last][3];

        velocityRingBufferCount++;

        velocity[0] = dX/dT;
        velocity[1] = dY/dT;
        velocity[2] = dH/dT;
    }

    public double[] getVelocity(){
        return velocity;
    }

    public abstract AprilTagProcessor getProcessor();
}



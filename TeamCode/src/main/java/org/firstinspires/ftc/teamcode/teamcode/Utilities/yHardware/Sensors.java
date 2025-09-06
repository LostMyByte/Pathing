package org.firstinspires.ftc.teamcode.teamcode.Utilities.yHardware;


import com.qualcomm.robotcore.util.ElapsedTime;


public class Sensors {

    private static long currentTimeMillis;
    public static long loopTime;
    private static long pastTime;
    public static ElapsedTime intakeTime = new ElapsedTime();


    public static void init(){
        pastTime = currentTimeMillis;
    }

    public static void update(){
        currentTimeMillis = System.currentTimeMillis();

        long deltaMili = currentTimeMillis - pastTime;
        pastTime = currentTimeMillis;
        loopTime = deltaMili;
    }

    public static long currentTimeMillis(){ return currentTimeMillis; }


}


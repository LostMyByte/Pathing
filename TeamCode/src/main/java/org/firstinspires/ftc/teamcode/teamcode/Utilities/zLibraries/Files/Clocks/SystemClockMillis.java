package org.firstinspires.ftc.teamcode.teamcode.Utilities.zLibraries.Files.Clocks;

public class SystemClockMillis implements Clock {
    private long startTime;
    public SystemClockMillis(){
        startTime = System.currentTimeMillis();
    }

    public long getCurrentTime(){
        return System.currentTimeMillis();
    }
    public double getTimePassed(){
        return (getCurrentTime() - startTime);
    }

}

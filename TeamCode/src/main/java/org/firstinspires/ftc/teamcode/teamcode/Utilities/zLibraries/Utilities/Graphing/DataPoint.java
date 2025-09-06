package org.firstinspires.ftc.teamcode.teamcode.Utilities.zLibraries.Utilities.Graphing;

public class DataPoint {

    private long timeStamp;
    private double dataPoint;
    private String unit;

    public DataPoint(long timeStamp, double dataPoint, String unit) {
        this.timeStamp = timeStamp;
        this.dataPoint = dataPoint;
        this.unit = unit;
    }

    public DataPoint(long timeStamp, double dataPoint){
        new DataPoint(timeStamp, dataPoint, "");
    }

    public DataPoint(double dataPoint){
        new DataPoint(0, dataPoint, "");
    }

    public long getTimeStamp() {
        return timeStamp;
    }

    public void setTimeStamp(long timeStamp) {
        this.timeStamp = timeStamp;
    }

    public double get() {
        return dataPoint;
    }

    public void set(double dataPoint) {
        this.dataPoint = dataPoint;
    }

    public String getUnit() {
        return unit;
    }

    public void setUnit(String unit) {
        this.unit = unit;
    }

    public String toString(){
        return dataPoint + " " + unit + " at " + timeStamp + "ms";
    }
}

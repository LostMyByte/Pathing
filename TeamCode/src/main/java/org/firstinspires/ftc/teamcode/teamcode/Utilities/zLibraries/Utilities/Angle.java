package org.firstinspires.ftc.teamcode.teamcode.Utilities.zLibraries.Utilities;

public class Angle {

    private double angle;
    private double offset;

    public Angle(double angle){
        this.angle = angle;
        offset = 0;
    }

    public Angle(double angle, double offset){
        this.offset = offset;
        this.angle = angle + offset;
    }

    public Angle(double angle, Direction direction){
        switch(direction){
            case RIGHT:
                offset = 0;
                break;
            case UP:
                offset = 90;
                break;
            case LEFT:
                offset = 180;
                break;
            case DOWN:
                offset = 270;
                break;
        }
        this.angle = angle + offset;
    }

    public double getDeg(){
        return angle - offset;
    }

    public double getRad(){
        return MathUtils.degsToRads(angle - offset);
    }

    public double sin(){
        return MathUtils.degSin(angle);
    }

    public double cos(){
        return MathUtils.degCos(angle);
    }

    public double tan(){
        return MathUtils.degTan(angle);
    }

    public enum Direction{
        RIGHT, UP, LEFT, DOWN
    }
}

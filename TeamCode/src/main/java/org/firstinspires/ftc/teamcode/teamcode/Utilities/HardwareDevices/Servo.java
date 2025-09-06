package org.firstinspires.ftc.teamcode.teamcode.Utilities.HardwareDevices;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;

import java.util.ArrayList;

public class Servo {

    com.qualcomm.robotcore.hardware.Servo servo;
    static ArrayList<Servo> allServos = new ArrayList<>();

    ArrayList<Servo> pairedServos = new ArrayList<>();

    double targetPosition = -1;
    double maxAngle = 999999, minAngle = -999999;
    double pointsPerDegree;
    double baseAngle;

    public Servo(String name, boolean reversed){
        allServos.add(this);

        servo = BaseOpMode.hardware.servo.get(name);

        if(reversed) servo.setDirection(com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE);
        else         servo.setDirection(com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD);

    }

    public Servo(String name, double point1, double angle1, double point2, double angle2, double maxAngle, double minAngle){
        allServos.add(this);

        servo = BaseOpMode.hardware.servo.get(name);

        setInterpolation(point1, angle1, point2, angle2, maxAngle, minAngle);
    }

    public Servo(String name, double point1, double angle1, double point2, double angle2){
        allServos.add(this);

        servo = BaseOpMode.hardware.servo.get(name);

        setInterpolation(point1, angle1, point2, angle2);
    }

    public Servo(String name){
        this(name, false);
    }

    public void setInterpolation(double point1, double angle1, double point2, double angle2, double maxAngle, double minAngle){
        pointsPerDegree = (point2 - point1) / (angle2 - angle1);
        baseAngle = point1 - angle1 * pointsPerDegree;
        this.maxAngle = maxAngle;
        this.minAngle = minAngle;
    }

    public void setInterpolation(double point1, double angle1, double point2, double angle2){
        setInterpolation(point1, angle1, point2, angle2, 9999999, -999999);
    }

    private void setInterpolation(double ppd, double ba){
        baseAngle = ba;
        pointsPerDegree = ppd;
    }

    public void pair(Servo s){
        pairedServos.add(s);
        s.targetPosition = targetPosition;
    }

    public void unpair(Servo s){
        pairedServos.remove(s);
    }

    public void setPosition(double p){
        targetPosition = p;
        for(Servo s : pairedServos){
            s.setPosition(p);
        }
    }

    public void setPositionInterpolated(double a){
        a = Range.clip(a, minAngle, maxAngle);

        double targetPosition = baseAngle + a * pointsPerDegree;

        this.targetPosition = targetPosition;

        for(Servo s : pairedServos){
            s.setPosition(targetPosition);
        }
    }

    private void commandPosition(){
        if(targetPosition != -1) servo.setPosition(targetPosition);
    }

    public static void commandPositions(){
        for(Servo s : allServos){
            s.commandPosition();
        }
    }

    public static void resetServoList(){
        allServos.clear();
    }


    public void setPower(double crServoPower) {
    }
}

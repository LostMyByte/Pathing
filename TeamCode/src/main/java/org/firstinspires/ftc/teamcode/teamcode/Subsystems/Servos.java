package org.firstinspires.ftc.teamcode.teamcode.Subsystems;


import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Servos.ServosDash.downPos1;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Servos.ServosDash.downPos2;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Servos.ServosDash.leftBetween;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Servos.ServosDash.leftClose;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Servos.ServosDash.leftOpen;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Servos.ServosDash.specimenClose;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Servos.ServosDash.specimenOpen;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.teamcode.Utilities.Dash.DashPositions;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.HardwareDevices.Servo;


public class Servos {


    @Config
    public static class ServosDash{

        public static double depositorArm0 = 0.32;
        public static double depositorArm90 = 0.1;
        public static double intakeV4b90 = 0;
        public static double intakeV4b0 = 0.45;
        public static double grabberOpenLeft = 0.2;
        public static double grabberCloseLeft = 0.1;
        public static double grabberOpenRight = 0.28;
        public static double grabberCloseRight = 0.18;
        public static double v4b0 = 0.5;
        public static double v4b90 = 0.92;
        public static double intakeWrist0 = 0.45;
        public static double intakeWrist90 = 0.1;
        public static double leftClose = 0.22;
        public static double leftOpen = 0.48;
        public static double rightClose = 0.58;
        public static double rightOpen = 0.3;

        public static double leftBetween = 0.34;
        public static double rightBetween = 0.4;
        public static double wrist90 = 0.7;
        public static double wrist0 = 0.5;
        public static double downPos1 = 1;
        public static double downPos2 = 0.15;
        public static double climbPos1 = 0.9;
        public static double climbPos2 = 0.6;
        public static double holdPlane = 0.7;
        public static double launchPlane = 1;
        public static double specimenOpen;
        public static double specimenClose;
        public static double intakeArmLeft0 = 0.76;
        public static double intakeArmLeft90 = 0.37;
        public static double intakeArmRight0 = 0.25;
        public static double intakeArmRight90 = 0.62;

        public static double intakeTurret45 = 0.84;
        public static double intakeTurret0 = 0.55;
    }

    public static double intakeV4bPos = 0;

    public static class DifferentialLeft extends Servo{
        public DifferentialLeft(){super(Hardware.differentialLeft);}
    }
    public static class DifferentialRight extends Servo{
        public DifferentialRight(){super(Hardware.differentialRight);}
    }

    public static class Hood extends Servo{
        public Hood(){
            super(Hardware.hood,0.9,25,0.62,65);
        }

        @Override
        public void setPosition(double p) {
            super.setPosition(p);
        }

        @Override
        public void setPositionInterpolated(double a) {
            super.setPositionInterpolated(a);
        }
    }


    //For everything, 0 is straight up, 90 is facing forwards towards the intake

    public static class DepositorArm extends Servo{
        //        ServoEncoder v4bEncoder;
        public DepositorArm(){
            super(Hardware.depositorArmRight, ServosDash.depositorArm0,0,ServosDash.depositorArm90,90);
        }
        public void setPosition(double target){
            setPositionInterpolated(target);
        }
        //        public void getPosition(){v4bEncoder.getPositionDegrees();}
        public void setPositionRaw(double target){
            super.setPosition(target);
        }
    }
    public static class DepositorArmLeft extends Servo{
        public DepositorArmLeft(){
            super(Hardware.deposotorArmLeft, 0.92, 90, 0.71, 0);
        }
        public void setPosition(double target){
            setPositionInterpolated(target);
        }
        //        public void getPosition(){v4bEncoder.getPositionDegrees();}
        public void setPositionRaw(double target){
            super.setPosition(target);
        }
    }

    public static class DepositorClaw extends Servo{
        public DepositorClaw(){
            super(Hardware.depositorClaw);
        }
        public void open(){this.setPosition(.3);}
        public void closed(){this.setPosition(.65);}
    }

    public static class IntakeArmLeft extends Servo{
        public IntakeArmLeft(){
            super(Hardware.intakeArmLeft, ServosDash.intakeArmLeft0, 0, ServosDash.intakeArmLeft90, 90);
        }
        public void setPositionRaw(double target){
            super.setPosition(target);
        }
        public void setPositionInterpolated(double target){super.setPositionInterpolated(target);}
    }
    public static class IntakeArmRight extends Servo{
        public IntakeArmRight(){
            super(Hardware.intakeArmRight, ServosDash.intakeArmRight0, 0, ServosDash.intakeArmRight90, 90);
        }
        public void setPositionRaw(double target){
            super.setPosition(target);
        }
        public void setPositionInterpolated(double target){super.setPositionInterpolated(target);}
    }



//    public static class IntakeDiffyRight extends Servo{
//        public IntakeDiffyRight(){
//            super(Hardware.intakeDiffyRight);
//        }
//    }
//    public static class IntakeDiffyLeft extends Servo{
//        public IntakeDiffyLeft(){
//            super(Hardware.intakeDiffyLeft);
//        }
//    }

    public static class BottomSweeper extends Servo{
        public BottomSweeper(){super(Hardware.bottomSweeper);}
        public void retracted(){this.setPosition(1);}
        public void extended(){this.setPosition(0.6);}
    }
    /*
    public static class IntakeTurret extends Servo{
        public IntakeTurret(){
            super(Hardware.intakeTurret, intakeTurret0,0,intakeTurret45,45);
        }
        public void setPositionRaw(double target){super.setPosition(target);}
        public void setPositionInterpolated(double target){super.setPositionInterpolated(target);}
    }

     */
    public static class DepositorDoor extends Servo{
        public DepositorDoor(){
            super(Hardware.depositorDoor);
        }
        public void open(){super.setPosition(DashPositions.servoTest);}
        public void close(){super.setPosition(DashPositions.servoTest);}
    }
    public static class GrabberLeft extends Servo{
        public GrabberLeft(String mapName){
            super(mapName, false);
        }

        public void grab(){
            setPosition(ServosDash.grabberOpenLeft);
        }

        public void letGo(){
            setPosition(ServosDash.grabberCloseLeft);
        }

    }
    public static class GrabberRight extends Servo{
        public GrabberRight(String mapName){
            super(mapName, false);
        }

        public void grab(){
            setPosition(ServosDash.grabberOpenRight);
        }

        public void letGo(){
            setPosition(ServosDash.grabberCloseRight);
        }

    }
    public static class V4BServo extends Servo{

        public V4BServo(){
            super("v4b", 0,90,  0.7,180);
        }

        public void setPosition(double target){
            setPositionInterpolated(target);
        }

        public void setPositionRaw(double target){
            super.setPosition(target);
        }

    }
    public static class RubberBandServo extends Servo {
        public RubberBandServo() {super("droneShooter");}}
    public static class ClawLeft extends Servo{
        public ClawLeft(){
            super(Hardware.clawLeft);
        }
        public void open(){
            setPosition(leftOpen);
        }
        public void close(){
            setPosition(leftClose);
        }

        public void transfer(){
            setPosition(leftBetween);
        }
    }

    public static class SpecimenClaw extends Servo{
        public SpecimenClaw(){
            super(Hardware.specimenClaw);
        }
        public void open(){
            setPosition(specimenOpen);
        }
        public void close(){
            setPosition(specimenClose);
        }

    }
    //    public static class ClawRight extends Servo{
//        public ClawRight(){
//            super(Hardware.clawRight);
//        }
//        public void open(){
//            setPosition(rightOpen);
//        }
//        public void close(){
//            setPosition(rightClose);
//        }
//
//        public void transfer(){
//            setPosition(rightBetween);
//        }
//    }
    public static class Climb1 extends Servo{
        public Climb1(String mapName){
            super(mapName);
        }

        public void start(){
            setPosition(downPos1);
        }

        public void climb(){
            setPosition(ServosDash.climbPos1);
        }
    }
    public static class Climb2 extends Servo{
        public Climb2(String mapName){
            super(mapName);
        }

        public void start(){
            setPosition(downPos2);
        }

        public void climb(){
            setPosition(ServosDash.climbPos2);
        }
    }

}

//brown is left - 1
//orange is on right - 3

//white is left - 0
//red is right - 4
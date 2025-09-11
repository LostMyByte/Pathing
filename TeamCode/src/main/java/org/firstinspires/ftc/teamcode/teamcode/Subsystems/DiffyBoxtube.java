package org.firstinspires.ftc.teamcode.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Control.PID;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Dash.PIDTuningDash;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.HardwareDevices.Motor;

public class DiffyBoxtube extends Subsystem{

    int loopNumber = 0;
    double angleIntegral;
    Motor BoxTubeL;
    Motor BoxTubeR;
    double voltage;
    TouchSensor limitSwitch;
    PID PivotPID;
    PID ExtensionPID;
    AnalogInput absoluteEncoder;
    double leftPosition;
    double rightPosition;
    double targetAngle;
    double targetExtension;
    double currentAngle;
    double currentExtension;
    double angleCorrection;
    double extensionCorrection;
    double extensionOffset;
    double midPoint;
    boolean finished;
    boolean extensionIn;
    boolean angleGood;
    boolean extensionOut;
    double momentOfInertiaConstant;
    public static double boxtubeMass = 1.1;
    public static double springConstant = 0.0147;
    public static double comXComponent = 0.054;
    public static double torqueConstantAngle = 9.179/9.2;
    public static double resistance = 12/9.2;

    public DiffyBoxtube(HardwareMap hardwareMap) {
        BoxTubeL = new Motor(Hardware.boxTubeL, false, true);
        BoxTubeR = new Motor(Hardware.boxTubeR, false, true);
        ExtensionPID = new PID(0,0,0);
        PivotPID = new PID(0, 0, 0);
        absoluteEncoder = hardwareMap.get(AnalogInput.class, "absoluteEncoder");

    }

    //This code will work in theory, but in practice, it will result in very inefficient paths, as it doesn't try to lower the momemt of interia of the arm as it rotates.
    public void work() {
        ExtensionPID.setConstants(PIDTuningDash.EP,PIDTuningDash.EI,PIDTuningDash.ED);
        PivotPID.setConstants(PIDTuningDash.AP,PIDTuningDash.AI,PIDTuningDash.AD);
        PivotPID.setFeedForward(PIDTuningDash.AFF);

        //i is the moment of inertia of the arm. This will be calculated with a regression, but this is a placeholder.
        double i = currentExtension * momentOfInertiaConstant;

        leftPosition = BoxTubeL.encoder.getPosition();
        rightPosition = BoxTubeR.encoder.getPosition();



        //uses the absolute encoder
        currentAngle = getCurrentAngle();

        //gets the midpoint between the 2 motor encoders, then gets how far motor's positions are from the midpoint
        currentExtension = getExtension();

        moveWithPID(targetExtension, targetAngle);
    }



    public void moveWithPID(double targetExtension, double targetAngle){
        targetExtension = Range.clip(targetExtension, 0, 1800);
        targetAngle = Range.clip(targetAngle, 0, 90);
        extensionCorrection = ExtensionPID.getCorrection(currentExtension, targetExtension);
        angleCorrection = PivotPID.getCorrection(currentAngle, targetAngle);

        //dividing by 12 to convert voltage to motor power
        angleCorrection += getHoldVoltage()/12;

        BoxTubeL.setPower(angleCorrection + extensionCorrection);
        BoxTubeR.setPower(angleCorrection - extensionCorrection);

        BaseOpMode.addData("extensionCorrection", extensionCorrection);
        BaseOpMode.addData("angleCorrection", angleCorrection);
        BaseOpMode.addData("currentExtension",currentExtension);
        BaseOpMode.addData("targetExtension", targetExtension);
        BaseOpMode.addData("currentAngle",currentAngle);
        BaseOpMode.addData("targetAngle", targetAngle);
        BaseOpMode.addData("holdPower", getHoldVoltage()/12);
    }

    public double getExtension(){
        BaseOpMode.addData("rightEncoder", rightPosition);
        BaseOpMode.addData("leftEncoder", leftPosition);

        //gets the midpoint between the 2 motor encoders, then gets how far motor's positions are from the midpoint
        currentExtension = ((leftPosition + rightPosition) / 2);
        currentExtension += extensionOffset;

        return currentExtension;
    }

    public double getExtensionM(){
        BaseOpMode.addData("rightEncoder", rightPosition);
        BaseOpMode.addData("leftEncoder", leftPosition);

        //gets the midpoint between the 2 motor encoders, then gets how far motor's positions are from the midpoint
        currentExtension = ((leftPosition + rightPosition) / 2);
        currentExtension += extensionOffset;

        //values obtained from regression
        //return 0.000336111*currentExtension+0.415;

        //because the encoders are returning questionable data, I am just going to return the minumum length
        return 0.415;
    }



    public void setTargetPosition(double angle, double length) {
        targetAngle = angle;
        targetExtension = length;
    }
//    public void updateOffset(){
//        if (limitSwitch.isPressed()){
//            extensionOffset = -currentExtension;
//        }
//    }

    public double getCurrentAngle(){
        //thank you avery for not letting me zero the absolute encoder and also for putting it on backwards

        loopNumber++;

        //resolution
        double voltage = absoluteEncoder.getVoltage() *72;
        //avery didn't zero it
        if (voltage < 75){
            voltage += 360;
        }
        voltage -= 279.900;

        //more resolution
        voltage = (voltage/150)*90;

        //avery put it on backwards
        double difference = 45-voltage;
        voltage = voltage + (2*difference);

        angleIntegral += voltage;
        BaseOpMode.addData("unfilteredAngle", voltage);
        if (loopNumber > 10){
            angleIntegral *= 10.0/11.0;
            BaseOpMode.addData("filteredAngle", angleIntegral/10);
            return  angleIntegral/10;
        } else {
            return voltage;
        }

    }

    //gets the torque necessary to hold the arm in its current position
    public double getHoldTorque(){
        double springTorque = -springConstant*(-currentAngle+90);
        double gravityTorque = boxtubeMass*Constants.g*getDistanceFromCenterOfMassToCenterOfRotation()*Math.cos(currentAngle);
        double holdTorque = -(gravityTorque+springTorque);
        BaseOpMode.addData("holdTorque", holdTorque);
        return holdTorque;
    }

    private double getHoldVoltage(){
        //this assumes that the motor is moving at zero velocity. If it has velocity from a previous movement, this will
        //provide less voltage than necessary to hold it in its position, thus slowing it down. In theory, this could make
        //the arm unstable when it isn't moving, but I haven't seen this happen yet, and I think gear lash might actually
        //be working in my favor here.

        //Note that I divide the result by 2 to account for the fact that there are two motors.
        return ((getHoldTorque()/torqueConstantAngle)*resistance)/2;
    }

    public double getDistanceFromCenterOfMassToCenterOfRotation(){
        //values obtained from regression
        double comYComponent = 0.201653*getExtensionM()+0.00431405;
        double distanceFromCenterOfMassToCenterOfRotation = Math.sqrt(Math.pow(comXComponent,2)+Math.pow(comYComponent,2));
        BaseOpMode.addData("distanceFromCenterOfMassToCenterOfRotation", distanceFromCenterOfMassToCenterOfRotation);
        return distanceFromCenterOfMassToCenterOfRotation;

    }

    public void update(){
        //updateOffset();
        work();
    }
    public void updateSensors(){}

    public enum BoxtubeMovementMode{
        THREE_POINT, TWO_POINT, DIRECT
    }
}

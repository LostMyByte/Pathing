package org.firstinspires.ftc.teamcode.teamcode.Utilities.Control;


import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;

public class PID {

    double kP, kI, kD;
    double feedForwardConstant = 0, lowerLimitConstant = 0, deadZone = 0, homedThreshold = 0, homedConstant = 0;

    double integralSum = 0;
    RingBuffer<Double> prevErrorRingBuffer = new RingBuffer<>(3,0.0);
    RingBuffer<Double> timeRingBuffer = new RingBuffer<>(3,0.0);

    ElapsedTime runtime = new ElapsedTime();

    public PID(double kP, double kI, double kD){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        reset();
    }

    public void setConstants(double kP, double kI, double kD){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setFeedForward(double constant){
        this.feedForwardConstant = constant;
    }

    public void setDeadZone(double deadZone){
        this.deadZone = deadZone;
    }

    public void setHomedThreshold(double homedThreshold){
        this.homedThreshold = homedThreshold;
    }

    public void setHomedConstant(double homedConstant){
        this.homedConstant = homedConstant;
    }

    public void setLowerLimit(double constant){
        this.lowerLimitConstant = constant;
    }

    public void reset(){
        prevErrorRingBuffer.fill(0.0);
        timeRingBuffer.fill(0.0);
        runtime.reset();
    }
    public double getCorrection(double error){

//        BaseOpMode.addData("Error", error);

//        if(target < homedThreshold && current < homedThreshold){
//            return homedConstant;
//        }

        double currentTime = runtime.milliseconds();
        double prevTime = timeRingBuffer.getValue(currentTime);
        double deltaTime = currentTime - prevTime;

        if(deltaTime > 200){
            reset();
            deltaTime = 0;
        }

//        BaseOpMode.addData("DeltaTime", deltaTime);


        double previousError = prevErrorRingBuffer.getValue(error);



        //Proportional component
        double pComponent = error * kP;

        //Integral component
        if(Math.signum(previousError) != Math.signum(error)){
            integralSum = 0;
        }

        integralSum += kI * error;

        double iComponent = integralSum * deltaTime;

        double dComponent = 0;
        //Derivative Component
        if (deltaTime != 0) {
             dComponent = (error - previousError) / deltaTime * kD;
        }


        //FeedForward Component
        double lowerLimitComponent = Math.signum(error) * lowerLimitConstant;

//        BaseOpMode.addData("Responce", pComponent + iComponent + dComponent + lowerLimitComponent + feedForwardConstant);

        if(Math.abs(error) < deadZone){
            integralSum = 0;
//            BaseOpMode.addData("FeedForward", error);
            return feedForwardConstant + pComponent + iComponent + dComponent;
        }

        return pComponent + iComponent + dComponent + lowerLimitComponent + feedForwardConstant;
    }
    public double getCorrection(double current, double target){
        double error = target - current;

//        BaseOpMode.addData("Error", error);

//        if(target < homedThreshold && current < homedThreshold){
//            return homedConstant;
//        }

        double currentTime = runtime.milliseconds();
        double prevTime = timeRingBuffer.getValue(currentTime);
        double deltaTime = currentTime - prevTime;

        if(deltaTime > 200){
            reset();
            deltaTime = 0;
        }

//        BaseOpMode.addData("DeltaTime", deltaTime);


        double previousError = prevErrorRingBuffer.getValue(error);



        //Proportional component
        double pComponent = error * kP;

        //Integral component
        if(Math.signum(previousError) != Math.signum(error)){
            integralSum = 0;
        }

        integralSum += deltaTime * error;

        double iComponent = integralSum * kI;

        //Derivative Component
        double dComponent = 0;
        if (deltaTime != 0) {
            dComponent = (error - previousError) / deltaTime * kD;
        }
        //FeedForward Component
        double lowerLimitComponent = Math.signum(error) * lowerLimitConstant;

//        BaseOpMode.addData("Responce", pComponent + iComponent + dComponent + lowerLimitComponent + feedForwardConstant);

        if(Math.abs(error) < deadZone){
            integralSum = 0;
//            BaseOpMode.addData("FeedForward", error);
            return feedForwardConstant + pComponent + iComponent + dComponent;
        }

        return pComponent + iComponent + dComponent + lowerLimitComponent + feedForwardConstant;
    }



    public double getCorrectionHeading(double current, double target){
        double error = target - current;

        while (error >  Math.PI){
            error -= 2 * Math.PI;
        }

        while (error < -Math.PI){
            error += 2 * Math.PI;
        }

        BaseOpMode.addData("error", error);

//        BaseOpMode.addData("Error", error);

//        if(target < homedThreshold && current < homedThreshold){
//            return homedConstant;
//        }

        double currentTime = runtime.milliseconds();
        double prevTime = timeRingBuffer.getValue(currentTime);
        double deltaTime = currentTime - prevTime;

        if(deltaTime > 200){
            reset();
            deltaTime = 0;
        }

//        BaseOpMode.addData("DeltaTime", deltaTime);


        double previousError = prevErrorRingBuffer.getValue(error);



        //Proportional component
        double pComponent = error * kP;

        //Integral component
        if(Math.signum(previousError) != Math.signum(error)){
            integralSum = 0;
        }

        integralSum += kI * error;

        double iComponent = integralSum * deltaTime;

        //Derivative Component
        double dComponent = (error - previousError) / deltaTime * kD;

        //FeedForward Component
        double lowerLimitComponent = Math.signum(error) * lowerLimitConstant;

//        BaseOpMode.addData("Responce", pComponent + iComponent + dComponent + lowerLimitComponent + feedForwardConstant);

        if(Math.abs(error) < deadZone){
            integralSum = 0;
//            BaseOpMode.addData("FeedForward", error);
            return feedForwardConstant + pComponent + iComponent + dComponent;
        }

        return pComponent + iComponent + dComponent + lowerLimitComponent + feedForwardConstant;
    }
}

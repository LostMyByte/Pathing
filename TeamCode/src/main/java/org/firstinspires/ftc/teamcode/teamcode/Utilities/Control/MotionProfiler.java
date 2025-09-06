package org.firstinspires.ftc.teamcode.teamcode.Utilities.Control;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;

@Config
public class MotionProfiler {

    // Kv is how many volts you need to move the motor to set speed
    // Should be V/(radius * angular velocity), then dividid by V_bat
    // Works out to be unitless, as it controls a PWM duty cycle.
    // Kv = 12V / r*rad/s /V_bat
    private double kV;
    private double lowerLimit;
    private double velP;

    private double maxAccel;
    private double maxVel;

    // NOTE: This value should be positive, even tough it is slowing down.
    private double maxDecel;
    private double maxDecelClose;
    public static double closeNessThreshold = 0.3;

    private double oldDistance = 0;
    private double oldTime = 0;

    private ElapsedTime timer;

    private boolean logging = false;

    private String loggingName = "Unnamed Motion Profile";

    public MotionProfiler(double kV, double lowerLimit, double velP, double maxAccel, double maxVel, double maxDecel, double maxDecelClose) {
        timer = new ElapsedTime();
        timer.reset();

        setConstants(kV, lowerLimit, velP, maxAccel, maxVel, maxDecel, maxDecelClose);
    }

    public void setConstants (double kV, double lowerLimit, double velP, double maxAccel, double maxVel, double maxDecel, double maxDecelClose) {
        this.kV = kV;
        this.lowerLimit= lowerLimit;
        this.velP = velP;
        this.maxAccel = maxAccel;
        this.maxVel = maxVel;
        this.maxDecel = maxDecel;
        this.maxDecelClose = maxDecelClose;
    }

    public void resetTimer() {
        // Call this when updating target position. Resets the timer used for ideal acceleration.
        timer.reset();

    }

    public double getCorrection(double error, double velocity) {
        return getCorrectionInternal(error, velocity);
    }

    private double getCorrectionInternal(double distance, double velocity) {
        // Internally calculate the correction, in case we want to expose other methods (e.g. using the timer for velocity)

        double time = timer.time();
        double deltaTime = time - oldTime;



        // Not using the velocity here because this is looking for sudden changes in target position
        // We have better methods of calculating velocity (e.g. goBuilda odometry computer)
        if (Math.abs(distance-oldDistance) / deltaTime - 100 > maxVel) {
            resetTimer();
            BaseOpMode.addData(loggingName, "Reset Acceleration");
        }


        double targetVelocity = calculateTargetVelocity(distance, timer.seconds()) * Math.signum(distance);
        double error = targetVelocity - velocity;

        oldDistance = distance;
        oldTime = time;

        double correction = error * velP + kV * targetVelocity + lowerLimit * Math.signum(distance);

        if (logging) {
            BaseOpMode.addData(loggingName + " current velocity", velocity);
            BaseOpMode.addData(loggingName + " target velocity", targetVelocity);
            BaseOpMode.addData(loggingName + " velocity error", error);
            BaseOpMode.addData(loggingName + " displacement", distance);
            BaseOpMode.addData(loggingName + " correction", correction);
        }

        return correction;
    }


    public void setTitle(String title) {
        this.loggingName = title;
    }

    public void setLogging(boolean logging) {
        this.logging= logging;
    }

    public double calculateTargetVelocity(double distance, double timeElapsed) {
        // Returns magnitude of correction - Direction should be towards target

        // Find velocity if it is just accelerating

        double accelerationVelocity = maxAccel * timeElapsed;

        // This gets the target velocity based off of position, so that the robot will always stop at the right point
        double decelerationVelocity = Math.abs(distance) > closeNessThreshold ? maxDecel * Math.sqrt(Math.abs(distance)/maxDecel)
                : maxDecelClose * Math.sqrt(Math.abs(distance)/maxDecelClose);

        // Get minimum of the three possible velocities
        if (accelerationVelocity < decelerationVelocity) {
            if (accelerationVelocity < maxVel) {
                if (logging) BaseOpMode.addData(loggingName + " is currently", "Accelerating");
                return accelerationVelocity;
            }
            else {
                if (logging) BaseOpMode.addData(loggingName + " is currently", "Holding");
                return maxVel;
            }
        }
        else if (decelerationVelocity < maxVel) {
            if (logging) BaseOpMode.addData(loggingName + " is currently", "Decelerating");
            return decelerationVelocity;
        }
        else {
            if (logging) BaseOpMode.addData(loggingName + " is currently", "Holding");
            return maxVel;
        }


    }

}

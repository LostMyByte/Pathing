// Primary Author: Kieran Mattingly
package org.firstinspires.ftc.teamcode.teamcode.Motion.Signals;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;

import java.util.ArrayList;

/**
 * A generic signal object for sensor/other data that varies over time.
 * Automatically generates derivatives, integrals, etc.
 */
public abstract class Signal {

    private static ElapsedTime timer;
    public static ArrayList<Signal> signals = new ArrayList<>();
    public static double deltaTime = 0.1;

    public int size;

    public Signal(int size) {
        signals.add(this);
        this.integralSum = Vector.length(size);
        this.data = Vector.length(size);
        this.oldData = Vector.length(size);
        this.size = size;
    }

    public static void startALl() {
        timer = new ElapsedTime();
        timer.reset();
    }

    public static void updateAll() {
        deltaTime = timer.seconds();
        timer.reset();
        BaseOpMode.addData("DeltaTime", deltaTime);
        for (Signal source : signals) {
            if (source.active) {
                source.telemetry();
                source.oldData = source.data;
                source.addIntegral();
                source.update();
            }
        }
    }

    // Add to integral using trapezoidal approximation
    protected void addIntegral() {
        this.integralSum.add(getDataVector().added(oldData).multiplied(0.5).multiplied(deltaTime));
    }

    protected Vector data;
    private Vector oldData;
    private Vector integralSum;


    protected abstract void update();

    public int getLength() {
        return data.length();
    }

    public double[] getData() {
        return data.getData();
    }

    public Vector getDataVector() {
        return data;
    }

    public Vector getGradient() {
        return data.subtracted(oldData).multiplied(1/timer.seconds());
    }

    public double[] getDerivatives() {
        return getGradient().getData();
    }

    public Vector getIntegralVector() {
        return integralSum;
    }

    public double[] getIntegrals() {
        return integralSum.getData();
    }

    private boolean active = true;

    public void setActive() {
        active = true;
        this.update();
    }

    public void setInactive() {
        active = false;
        this.oldData = null;
    }

    public void resetIntegral() {
        this.integralSum = Vector.length(data.length());
    }

    public abstract void telemetry();

}

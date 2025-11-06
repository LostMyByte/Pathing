package org.firstinspires.ftc.teamcode.teamcode.Motion.Signals;

import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;

/**
 * A signal that is one component of another signal.
 */
public class PartialSignal extends Signal {

    int index = 0;
    Signal source;
    public PartialSignal(int index, Signal other) {
        super(1);
        this.index = index;
        source = other;
    }

    @Override
    protected void update() {

    }

    public double getIntegral() {
        return source.getIntegralVector().get(index);
    }

    @Override
    public Vector getIntegralVector() {
        return new Vector(getIntegral());
    }

    public double getDataDouble() {
        return source.getDataVector().get(index);
    }

    @Override
    public Vector getDataVector() {
        return new Vector(getDataDouble());
    }

    public double getDerivative() {
        return source.getGradient().get(index);
    }

    @Override
    public Vector getGradient() {
        return new Vector(getDerivative());
    }


    @Override
    public void telemetry() {

    }
}

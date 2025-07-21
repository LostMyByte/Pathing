package org.firstinspires.ftc.teamcode.Motion.Controllers;

import org.firstinspires.ftc.teamcode.Utilities.LinearAlgebra.Vector;

public class DifferenceSignal extends Signal {

    public Signal signalA;
    public Signal signalB;

    @Override
    protected void update() {
        this.data = signalA.getDataVector().subtracted(signalB.getDataVector());
    }

    public DifferenceSignal(Signal signalA, Signal signalB) {
        super(signalA.size);
        assert signalA.size == signalB.size;

        this.signalA = signalA;
        this.signalB = signalB;
    }

    @Override
    public Vector getGradient() {
        return signalA.getGradient().subtracted(signalB.getGradient());
    }

    @Override
    public Vector getIntegralVector() {
        return signalA.getDataVector().subtracted(signalB.getIntegralVector());
    }

    @Override
    public void resetIntegral() {
        signalA.resetIntegral();
        signalB.resetIntegral();
    }
}

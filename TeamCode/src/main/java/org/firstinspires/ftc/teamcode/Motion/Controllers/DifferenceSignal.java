package org.firstinspires.ftc.teamcode.Motion.Controllers;

import org.firstinspires.ftc.teamcode.AAAOpModes.BaseOpMode;
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
        return signalA.getIntegralVector().subtracted(signalB.getIntegralVector());
    }

    @Override
    public void resetIntegral() {
        signalA.resetIntegral();
        signalB.resetIntegral();
    }

    @Override
    public void telemetry() {
        BaseOpMode.addData("Velocity Error X", data.getData()[0]);
        BaseOpMode.addData("Velocity Error Y", data.getData()[1]);
        BaseOpMode.addData("Velocity Error H", data.getData()[2]);

        BaseOpMode.addData("Position Error X", getIntegralVector().get(0));
        BaseOpMode.addData("Position Error Y", getIntegralVector().get(1));
        BaseOpMode.addData("Position Error H", getIntegralVector().get(2));
    }
}

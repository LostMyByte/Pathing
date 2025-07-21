package org.firstinspires.ftc.teamcode.Motion.Controllers;

import org.firstinspires.ftc.teamcode.Utilities.LinearAlgebra.Vector;

public abstract class Controller {

    public Signal referenceSignal;
    public Signal sensorSignal;
    public abstract Vector getCorrection();

    protected Signal errorSignal;

    public Controller(Signal referenceSignal, Signal dataSignal) {
        this.referenceSignal = referenceSignal;
        this.sensorSignal = dataSignal;
        errorSignal = new DifferenceSignal(referenceSignal, dataSignal);
    }

}

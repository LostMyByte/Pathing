package org.firstinspires.ftc.teamcode.Motion.Controllers;

import org.firstinspires.ftc.teamcode.Utilities.Math.Vector;

public abstract class Controller {

    public ReferenceSignal referenceSignal;
    public Signal sensorSignal;
    public abstract Vector getCorrection();

    protected int dimensions;

    protected Signal errorSignal;

    public Controller(ReferenceSignal referenceSignal, Signal dataSignal) {
        this.referenceSignal = referenceSignal;
        this.sensorSignal = dataSignal;
        errorSignal = new DifferenceSignal(referenceSignal, dataSignal);

        this.dimensions = errorSignal.size;
    }

}

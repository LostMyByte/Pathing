// Primary Author: Kieran Mattingly
package org.firstinspires.ftc.teamcode.teamcode.Motion.Controllers;

import org.firstinspires.ftc.teamcode.teamcode.Motion.Signals.DifferenceSignal;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Signals.ReferenceSignal;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Signals.Signal;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;
/**
 * A class to generate a control system based off reference and sensor signals.
**/
public abstract class Controller {

    public ReferenceSignal referenceSignal;
    public Signal sensorSignal;
    public abstract Vector getCorrection();

    protected int dimensions;

    protected Signal errorSignal;

    public Controller(){}

    public Controller(ReferenceSignal referenceSignal, Signal dataSignal) {
        this.referenceSignal = referenceSignal;
        this.sensorSignal = dataSignal;
        errorSignal = new DifferenceSignal(referenceSignal, dataSignal);

        this.dimensions = errorSignal.size;
    }

    public Vector targetPositionError() {
        return this.sensorSignal.getDataVector().subtracted(this.referenceSignal.target());
    }

}

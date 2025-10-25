package org.firstinspires.ftc.teamcode.teamcode.Motion.Signals;

import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;

public abstract class ReferenceSignal extends Signal {

    public ReferenceSignal(int size) {
        super(size);
    }

    /** Predicts the value of the signal some point in the future.
    * @param    time   time in the future to predict (seconds)
    * @return          predicted state
    */

    public abstract Vector predict(double time);

    public abstract Vector target();

    

}

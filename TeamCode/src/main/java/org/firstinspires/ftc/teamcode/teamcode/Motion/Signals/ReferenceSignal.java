// Primary Author: Kieran Mattingly
package org.firstinspires.ftc.teamcode.teamcode.Motion.Signals;

import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;

/**
 * A predefined signal that can be predicted.
 */
public abstract class ReferenceSignal extends Signal {

    public ReferenceSignal(int size) {
        super(size);
    }

    /** Predicts the value of the signal some point in the future.
    * @param    time   time in the future to predict (seconds)
    * @return          predicted state
    */
    public abstract Vector predict(double time);

    /** The signal target state (limit of the signal as time goes to infinity)
     * @return          target state
     */
    public abstract Vector target();

    

}

// Primary Author: Kieran Mattingly
package org.firstinspires.ftc.teamcode.teamcode.Motion.Signals.Filters;

import org.firstinspires.ftc.teamcode.teamcode.Motion.Signals.Signal;

/**
 * A low-pass filter for a generic signal.
 * TODO: Handle signals with custom derivative, integral methods (e.g. location) better.
 */
public class LowPassFilter extends Signal {

    Signal source;
    double alpha; // Parameter for the filter

    public LowPassFilter(Signal source, double alpha) {
        super(source.getLength());

        this.source = source;
        this.alpha = alpha;
    }

    @Override
    protected void update() {
        this.data.add(source.getDataVector().subtracted(this.data).multiplied(alpha));
    }

    @Override
    public void telemetry() {

    }
}

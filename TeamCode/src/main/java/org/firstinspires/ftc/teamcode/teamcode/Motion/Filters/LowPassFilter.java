package org.firstinspires.ftc.teamcode.teamcode.Motion.Filters;

import org.firstinspires.ftc.teamcode.teamcode.Motion.Controllers.Signal;

public class LowPassFilter extends Signal {

    Signal source;
    double alpha;

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

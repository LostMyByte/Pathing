package org.firstinspires.ftc.teamcode.Motion.Controllers;

import org.firstinspires.ftc.teamcode.Utilities.Math.Vector;

public class ConstantSignal extends ReferenceSignal {

    @Override
    protected void update() {

    }

    @Override
    public void telemetry() {

    }

    @Override
    public Vector predict(double time) {
        return this.data;
    }

    public ConstantSignal(Vector target) {
        super(target.length());
        this.data = target;
    }

    public Vector target() {
        return this.data;
    }
}

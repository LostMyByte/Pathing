// Primary Author: Kieran Mattingly
package org.firstinspires.ftc.teamcode.teamcode.Motion.Signals;

import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;

/**
 * A constant signal.
 */
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

    public ConstantSignal(double target) {
        super(1);
        this.data = new Vector(target);
    }
    public ConstantSignal(double x, double y) {
        super(2);
        this.data = new Vector(x, y);
    }

    public ConstantSignal(double x, double y, double z) {
        super(3);
        this.data = new Vector(x, y, z);
    }

    public Vector target() {
        return this.data;
    }
}

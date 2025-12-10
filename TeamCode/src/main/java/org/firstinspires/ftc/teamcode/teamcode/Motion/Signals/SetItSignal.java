package org.firstinspires.ftc.teamcode.teamcode.Motion.Signals;

import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;

public class SetItSignal extends Signal {

    public SetItSignal(int size) {
        super(size);
    }

    public void set(Vector val) {
        this.data = val;
    }

    public void set(double val) {
        this.data = new Vector(val);
    }

    @Override
    protected void update() {

    }
}

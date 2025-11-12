package org.firstinspires.ftc.teamcode.teamcode.Motion.Signals;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;

public class SequentialSignal extends ReferenceSignal {

    double switchtime;
    ElapsedTime timer;
    ReferenceSignal signal1;
    ReferenceSignal signal2;

    public SequentialSignal(ReferenceSignal signal1, ReferenceSignal signal2, double switchtime) {
        super(signal1.size);
        timer = new ElapsedTime();
        this.signal1 = signal1;
        this.signal2 = signal2;
        this.switchtime = switchtime;
    }

    @Override
    protected void update() {
        this.data = timer.time() > switchtime ? signal1.data : signal2.data;
    }

    @Override
    public void telemetry() {

    }

    @Override
    public Vector predict(double time) {
        return time > switchtime? signal2.predict(time - switchtime) : signal1.predict(time);
    }

    @Override
    public Vector target() {
        return signal2.target();
    }
}

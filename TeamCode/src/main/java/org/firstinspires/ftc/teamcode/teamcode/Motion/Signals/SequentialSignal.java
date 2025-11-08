package org.firstinspires.ftc.teamcode.teamcode.Motion.Signals;

import com.qualcomm.robotcore.util.ElapsedTime;

public class SequentialSignal extends Signal {

    double switchtime;
    ElapsedTime timer;
    Signal signal1;
    Signal signal2;

    public SequentialSignal(Signal signal1, Signal signal2, double switchtime) {
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
}

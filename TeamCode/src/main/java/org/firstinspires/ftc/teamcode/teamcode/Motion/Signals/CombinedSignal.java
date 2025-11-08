package org.firstinspires.ftc.teamcode.teamcode.Motion.Signals;

public class CombinedSignal extends Signal{
    Signal signal1;
    Signal signal2;
    public CombinedSignal(Signal signal1, Signal signal2) {
        super(signal1.size + signal2.size);
        this.signal1 = signal1;
        this.signal2 = signal2;
    }

    @Override
    protected void update() {
        double[] data = new double[size];
        for (int i = 0; i < signal1.size; i++) {
            data[i] = signal1.getData()[i];
        }
        for (int i = signal1.size; i < size; i++) {
            data[i] = signal2.getData()[i];
        }
    }

    @Override
    public void telemetry() {

    }
}

package org.firstinspires.ftc.teamcode.teamcode.Utilities.zLibraries.Utilities.Graphing;

import org.firstinspires.ftc.teamcode.teamcode.Utilities.zLibraries.Utilities.RingBuffer;

public class SpikeAnalyzerSimple {

    boolean isSpikeOccuring;
    boolean isSpikeOccuringPrev;
    boolean hasSpikeOccured;

    RingBuffer<Double> dataset;

    double extremium;
    double spikeStart;

    double spikeThresh;
    double median;

    double ringFilled;

    public SpikeAnalyzerSimple(int length, double spikeThresh) {
        this.dataset = new RingBuffer<Double>(length, 0.0);
        this.spikeThresh = spikeThresh;
    }

    public void update(double newDataPoint){
        double oldestVal = dataset.getValue(newDataPoint);

    }
}

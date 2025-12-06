package org.firstinspires.ftc.teamcode.teamcode.Motion.Signals.SensorModels;

import org.firstinspires.ftc.teamcode.teamcode.Motion.Signals.Signal;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Matrix;

public interface SensorModel {
    Vector toSensorSpace(Signal data);
    Vector predict(Vector state);

    Matrix getNoiseMatrix();

    Matrix getDerivative(Vector state);
}

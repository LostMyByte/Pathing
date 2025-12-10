package org.firstinspires.ftc.teamcode.teamcode.Motion.Signals.Filters;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains.SystemModels.SystemModel;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Signals.SensorModels.SensorModel;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Signals.Signal;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.SensorNoise;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.GeneralMatrix;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Matrix;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;

public class SensorFusionEKF extends Signal {
    SystemModel model;

    protected SensorModel[] sensorModels;

    protected Matrix P = new GeneralMatrix(5,5);
    protected Matrix covarience;
    protected Vector state;

    int numSensors;

    protected Signal[] sources;

    public static Vector input = new Vector(0,0);

    public SensorFusionEKF(Vector start, Signal[] sources, SensorModel[] sensors, SystemModel model) {
        super(3);
        this.state = start;
        this.sensorModels = sensors;
        this.sources = sources;
        this.model = model;
        this.numSensors = sensors.length;
    }

    @Override
    protected void update() {
        Vector xhat = model.stateTransitionFunction(state, input, Signal.deltaTime);
        Matrix F = model.dFdX(state, input, Signal.deltaTime);
        P = F.multiplied(P).multiplied(F.transposed()).added(covarience);
        Vector[] y = new Vector[numSensors];
        Matrix[] H = new Matrix[numSensors];
        Matrix[] S = new Matrix[numSensors];
        Matrix[] K = new Matrix[numSensors];


        for (int i = 0; i < numSensors; i ++) {
            Matrix alpha = Matrix.identityMatrix(5);
            y[i] = sensorModels[i].toSensorSpace(sources[i]).subtracted(sensorModels[i].predict(state));
            H[i] = sensorModels[i].getDerivative(xhat);
            S[i] = H[i].multiplied(P).multiplied(H[i].transposed()).added(sensorModels[i].getNoiseMatrix());
            Matrix Sinv = S[i].inverted();
            K[i] = P.multiplied(H[i].transposed()).multiplied(Sinv);
            xhat.add(K[i].multiplied(y[i]));
            alpha.subtract(K[i].multiplied(H[i]));
            P = alpha.multiplied(P);
        }

        state = xhat;

        this.data = new Vector(state.get(0), state.get(1), state.get(2));
    }

    @Override
    public Vector getGradient() {
        return new Vector(state.get(3) * -Math.sin(state.get(2)), state.get(3) * Math.cos(state.get(2)), state.get(4));
    }




}

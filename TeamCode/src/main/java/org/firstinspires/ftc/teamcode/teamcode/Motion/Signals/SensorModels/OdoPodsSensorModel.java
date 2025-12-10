package org.firstinspires.ftc.teamcode.teamcode.Motion.Signals.SensorModels;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.teamcode.Motion.Signals.Signal;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.SensorNoise;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.GeneralMatrix;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Matrix;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;

public class OdoPodsSensorModel implements SensorModel {



    @Override
    public Vector toSensorSpace(Signal data) {
        Vector result = Vector.length(6);
        result.put(0, data.getData()[0]);
        result.put(1, data.getData()[1]);
        result.put(2, data.getData()[2]);
        result.put(3, data.getDerivatives()[0]);
        result.put(4, data.getDerivatives()[1]);
        result.put(5, data.getDerivatives()[2]);
        return result;
    }


    // TODO: Make browninan, not gaussian
    @Override
    public Vector predict(Vector state) {
        return new Vector(
                state.get(0),
                state.get(1),
                state.get(2),
                state.get(3) * -Math.sin(state.get(2)),
                state.get(3) * Math.cos(state.get(2)),
                state.get(4)
        );
    }

    @Override
    public Matrix getNoiseMatrix() {
        return new GeneralMatrix(6, 6, new double[] {
                SensorNoise.odo_pos, 0, 0, 0, 0, 0,
                0, SensorNoise.odo_pos, 0, 0, 0, 0,
                0, 0, SensorNoise.odo_h, 0, 0, 0,
                0, 0, 0, SensorNoise.odo_v, 0, 0,
                0, 0, 0, 0, SensorNoise.odo_v, 0,
                0, 0, 0, 0, 0, SensorNoise.odo_vh
        });
    }

    @Override
    public Matrix getDerivative(Vector state) {
        return new GeneralMatrix(6, 5, new double[] {
                1, 0, 0, 0, 0,
                0, 1, 0, 0, 0,
                0, 0, 1, 0, 0,
                0, 0, -state.get(3)*Math.cos(state.get(2)), -Math.sin(state.get(2)), 0,
                0, 0, -state.get(3)*Math.sin(state.get(2)), Math.cos(state.get(2)), 0,
                0, 0, 0, 0, 1,
        });
    }
}

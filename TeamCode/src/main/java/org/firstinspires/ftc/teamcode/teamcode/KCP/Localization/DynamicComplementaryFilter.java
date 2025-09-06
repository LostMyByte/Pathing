package org.firstinspires.ftc.teamcode.teamcode.KCP.Localization;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.LinearAlgebra.Matrix;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.LinearAlgebra.Vector;


// A complementary filter that will dynamically change into a high-pass filter if the low-frequency data is unavailible (null);
public class DynamicComplementaryFilter {
    private Vector position;
    public Matrix error;

    private FtcDashboard dash;
    private Telemetry telemetry;
    private Vector oldLow; //SPKFN Position at last april tag detection
    private Vector oldHigh; //APRLTG Position at last april tag detection



    public DynamicComplementaryFilter(Matrix error, Vector start) {
        this.error=error;
        this.position=start;

        oldLow = start;
        oldHigh = start;

        dash = FtcDashboard.getInstance();
        telemetry = dash.getTelemetry();
    }


    public Vector getPrediction() {
        return position;
    }

    public double[] getPosition () {
        return new double[] {position.get(0), position.get(1)};
    }

    public double getAngle() {return position.get(2);}

    public void update(@NonNull Vector posLow, Vector posHigh) {

        BaseOpMode.addData("SparkFun X", posLow.get(0));
        BaseOpMode.addData("SparkFun Y", posLow.get(1));

        if (posHigh != null) {
            BaseOpMode.addData("AprilTags X", posHigh.get(0));
            BaseOpMode.addData("AprilTags Y", posHigh.get(1));
        }

        // May want to have it only use interpolated, not the Sparkfun as well
        position.add((Matrix.identityMatrix(error.numCols()).subtracted(error)).multiplied((posLow.subtracted(position))));

        if (posHigh != null) {
            oldHigh = posHigh;
            oldLow = posLow;
        }
        else {
            posHigh = oldHigh.added(posLow.subtracted(oldLow));
        }

        position.add(error.multiplied((posHigh.subtracted(position))));

        BaseOpMode.addData("Guessed X", position.get(0));
        BaseOpMode.addData("Guessed Y", position.get(1));

        //position = posHigh;

    }


    public void setState(Vector position) {
        this.position = position;
    }
}

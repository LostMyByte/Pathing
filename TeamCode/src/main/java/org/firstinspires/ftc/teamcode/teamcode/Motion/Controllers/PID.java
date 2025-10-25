package org.firstinspires.ftc.teamcode.teamcode.Motion.Controllers;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;

public class PID extends Controller {


    PIDCoefficients coeffs;

    @Override
    public Vector getCorrection() {

        Vector correction = new Vector(new double[errorSignal.getLength()]);

        correction.add(errorSignal.getDataVector().multiplied(coeffs.kP));
        correction.add(errorSignal.getIntegralVector().multiplied(coeffs.kI));
        correction.add(errorSignal.getGradient().multiplied(coeffs.kD));
        correction.add(referenceSignal.getDataVector().multiplied(coeffs.kF));
        correction.add(errorSignal.getDataVector().normalized().multiplied(coeffs.kL));

        if (coeffs.kC != null) correction.add(coeffs.kC);

        BaseOpMode.addData("Correction", correction.getData().toString());

        return correction;
    }

    public PID(ReferenceSignal referenceSignal, Signal dataSignal, PIDCoefficients coefficients) {
        super(referenceSignal, dataSignal);
        this.coeffs = coefficients;
    }
}

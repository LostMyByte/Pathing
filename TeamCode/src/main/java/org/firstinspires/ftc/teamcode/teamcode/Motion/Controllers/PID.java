// Primary Author: Kieran Mattingly
package org.firstinspires.ftc.teamcode.teamcode.Motion.Controllers;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Signals.ReferenceSignal;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Signals.Signal;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;

/**
 * A Basic PID (Proportional - Integral - Derivative) Controller
 * Also handles Feedforward, Constant, and Loopback terms.
 */
public class PID extends Controller {

    PIDCoefficients coeffs;

    @Override
    public Vector getCorrection() {


        Vector correction = new Vector(new double[errorSignal.getLength()]);

        if (errorSignal.getDataVector().magnitude() > coeffs.deadzone) {
            correction.add(errorSignal.getDataVector().multiplied(coeffs.kP));
            correction.add(errorSignal.getIntegralVector().multiplied(coeffs.kI));
            correction.add(errorSignal.getGradient().multiplied(coeffs.kD));
        }
        correction.add(referenceSignal.getDataVector().multiplied(coeffs.kF));
        correction.add(errorSignal.getDataVector().normalized().multiplied(coeffs.kL));

        if (coeffs.kC != null) correction.add(coeffs.kC);

        BaseOpMode.addData("Correction", correction.getData().toString());

        return correction;
    }

    public PID(Signal referenceSignal, Signal dataSignal, PIDCoefficients coefficients) {
        super(referenceSignal, dataSignal);
        this.coeffs = coefficients;
    }

    public static class PIDCoefficients {
        public double kP;
        public double kI;
        public double kD;
        public double kF;
        public Vector kC;
        public double kL;
        public double deadzone;

        public PIDCoefficients(double kP, double kI, double kD, double kF, double kL) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kF = kF;
            this.kL = kL;
        }
    }
}

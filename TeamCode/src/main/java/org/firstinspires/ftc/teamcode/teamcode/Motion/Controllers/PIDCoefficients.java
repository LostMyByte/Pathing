package org.firstinspires.ftc.teamcode.teamcode.Motion.Controllers;

import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;

// Should these be matricies?
public class PIDCoefficients {
    public double kP;
    public double kI;
    public double kD;
    public double kF;
    public Vector kC;
    public double kL;

    public PIDCoefficients(double kP, double kI, double kD, double kF, double kL) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.kL = kL;
    }
}

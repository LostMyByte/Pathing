package org.firstinspires.ftc.teamcode.teamcode.Motion.Paths;

import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Matrix;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;

public abstract class Obstacle {

    public abstract double getCost(Vector position);
    public abstract Vector getDerivative(Vector position);
    public abstract Matrix get2ndDerivative(Vector positon);
}

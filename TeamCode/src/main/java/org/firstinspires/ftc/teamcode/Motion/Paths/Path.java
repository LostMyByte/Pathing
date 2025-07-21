package org.firstinspires.ftc.teamcode.Motion.Paths;

import org.firstinspires.ftc.teamcode.Utilities.LinearAlgebra.Vector;

public abstract class Path {

    public abstract Vector getPosition(double t);
    public abstract Vector getVelocity(double t);
    public abstract Vector getAcceleration(double t);

    public abstract Vector getTarget();

}

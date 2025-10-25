package org.firstinspires.ftc.teamcode.teamcode.Motion.Paths;

import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;

public abstract class Path {

    public abstract Vector getPosition(double t);
    public abstract Vector getVelocity(double t);
    public abstract Vector getAcceleration(double t);

    public abstract Vector getTarget();

}

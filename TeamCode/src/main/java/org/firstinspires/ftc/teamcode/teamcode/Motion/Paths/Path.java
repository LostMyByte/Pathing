// Primary Author: Kieran Mattingly
package org.firstinspires.ftc.teamcode.teamcode.Motion.Paths;

import org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains.SystemModels.SystemModel;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Signals.ReferenceSignal;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Signals.Signal;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;

public abstract class Path {

    public abstract Vector getPosition(double t);
    public abstract Vector getVelocity(double t);

    public abstract Vector getTarget();
    public abstract Vector getTargetVelocity();



}

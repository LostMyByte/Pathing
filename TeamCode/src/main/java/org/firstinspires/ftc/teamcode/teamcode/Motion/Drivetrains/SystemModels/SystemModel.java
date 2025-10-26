// Primary Author: Kieran Mattingly
package org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains.SystemModels;

import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Matrix;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;

public interface SystemModel {

    Vector controlLimit(Vector u);

    Vector stateTransitionFunction(Vector vector, Vector currentControl, double dt);

    Matrix dFdX(Vector state, Vector control, double dt);

    Matrix dFdU(Vector state, Vector control, double dt);

    Matrix dSdU(Vector control);

    Matrix h(Vector state);

    Matrix VdF2dXdX(Vector state, Vector control, Vector vx, double dt);

    Matrix VdF2dXdU(Vector state, Vector control, Vector vx, double dt);

    Matrix VdF2dUdU(Vector state, Vector control, Vector vx, double dt);
}

// Primary Author: Kieran Mattingly
package org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains.SystemModels;

import org.firstinspires.ftc.teamcode.teamcode.Motion.Signals.Signal;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Matrix;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;

public interface SystemModel {

    int getDimensions();
    int getControls();

    /** Converts from a standard state into state space.
     * @param data Sensor data signal
     */
    Vector toStateSpace(Signal data);

    Vector toStateSpace(Vector data, Vector gradient);

    /**
     * Control Limiting Function.
     * Control will be adapted based off this function, so can be squashed from (-inf, inf) to a set range by a signmoid.
     * @param u     Control
     * @return      Limited Control
     */
    Vector controlLimit(Vector u);

    /**
     * Given the current state, control, and timestep, what will the next state be?
     * @param state             Current State
     * @param currentControl    Current Control
     * @param dt                Timestep
     * @return                  Next State
     */
    Vector stateTransitionFunction(Vector state, Vector currentControl, double dt);

    /**
     * Derivative of state transition function with respect to state
     * @param state     Current state
     * @param control   Current Control
     * @param dt        Timestep
     * @return          Derivative as a Jacobian.
     */
    Matrix dFdX(Vector state, Vector control, double dt);

    /**
     * Derivative of state transition function with respect to control.
     * @param state     Current state
     * @param control   Current Control
     * @param dt        Timestep
     * @return          Derivative as a Jacobian.
     */
    Matrix dFdU(Vector state, Vector control, double dt);

    /**
     * Derivative of control limiter with respect to state
     * @param control   Current Control
     * @return          Derivative as a Jacobian.
     */
    Matrix dSdU(Vector control);

    /**
     * Heading Transformation generator.
     * The model assumes the dynamics are linear with a single transform, usually heading.
     * This function generates a matrix that converts the linear model to the actual model.
     * @param state     Current state
     * @return          Heading transformation as a Matrix.
     */
    Matrix h(Vector state);

    /**
     * Multiply a vector by the second derivative of the cost function with respect to state.
     * This is a tensor contraction, which the libraries currently in use don't support.
     * Given that most of the time these are sparse tensors, it's feasible to implement the
     * multiplication on a case-by-case basis, however this should be improved in the future.
     * @param state     Current State
     * @param control   Current Control
     * @param vx        Vector to multiply by
     * @param dt        Timestep
     * @return          The tensor-vector product as a matrix
     */
    Matrix VdF2dXdX(Vector state, Vector control, Vector vx, double dt);

    /**
     * Multiply a vector by the second derivative of the cost function with respect to state and then control.
     * This is a tensor contraction, which the libraries currently in use don't support.
     * Given that most of the time these are sparse tensors, it's feasible to implement the
     * multiplication on a case-by-case basis, however this should be improved in the future.
     * @param state     Current State
     * @param control   Current Control
     * @param vx        Vector to multiply by
     * @param dt        Timestep
     * @return          The tensor-vector product as a matrix
     */
    Matrix VdF2dXdU(Vector state, Vector control, Vector vx, double dt);

    /**
     * Multiply a vector by the second derivative of the cost function with respect to control.
     * This is a tensor contraction, which the libraries currently in use don't support.
     * Given that most of the time these are sparse tensors, it's feasible to implement the
     * multiplication on a case-by-case basis, however this should be improved in the future.
     * @param state     Current State
     * @param control   Current Control
     * @param vx        Vector to multiply by
     * @param dt        Timestep
     * @return          The tensor-vector product as a matrix
     */
    Matrix VdF2dUdU(Vector state, Vector control, Vector vx, double dt);
}

// Primary Author: Kieran Mattingly
package org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains.SystemModels;


import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Signals.Signal;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.DriveWheels;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.GeneralMatrix;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Matrix;

import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;


import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.TrigAngle;

//import org.flag4j.arrays.dense.Matrix;
//import org.flag4j.arrays.dense.Vector;

public class MechanumDrive implements SystemModel {
    // Uses a state-space of [x, y, h, vx, vy, vh]
    // Model is xdot = h(x)(Ax + Bu), where
    //      xdot is the derivative of state with respect to time,
    //      x is the state,
    //      u is the control input

    // Converts from target powers to wheel powers
    public static Matrix W = new GeneralMatrix(4, 3, new double[] {
            DriveWheels.FR.y, DriveWheels.FR.x, DriveWheels.FR.h,
            DriveWheels.FL.y, DriveWheels.FL.x, DriveWheels.FL.h,
            DriveWheels.BR.y, DriveWheels.BR.x, DriveWheels.BR.h,
            DriveWheels.BL.y, DriveWheels.BL.x, DriveWheels.BL.h,
    });
    // I got the 0.25 by math. It's almost orthagonal.
    public static Matrix WL = W.transposed().multiplied(0.25);

    // Derivative of state due to current state.
    private Matrix A = new GeneralMatrix(  6, 6, new double[] {
            // The velocity is the derivative of position.
            // Assumes when h=0, velocity in the direction the robot is facing is in the y direction. This is then fixed by the heading transform.
            0 ,0, 0, 1, 0, 0,
            0 ,0, 0, 0, 1, 0,
            0 ,0, 0, 0, 0, 1,

            // The acceleration is drag forces due to backEMF.
            // Friction is handled separately.
            0 ,0, 0, DriveWheels.Ex, 0, 0,
            0 ,0, 0, 0, DriveWheels.Ey, 0,
            0 ,0, 0, 0, 0, DriveWheels.Eh,
    });

    // Control response matrix
    private Matrix B = new GeneralMatrix(6, 3, new double[] {
            0 ,0, 0,
            0 ,0, 0,
            0 ,0, 0,
            // A control affects the PWM duty cycle, which is directly proportional to the torque by the motor.
            // Given that the mass and moment of inertia are (for the most part) constant, the acceleration is
            // directly proportional to motor powers. These values are the scalars that form that proportionality.
            0, DriveWheels.XdriveAcceleration, 0,
            DriveWheels.YdriveAcceleration, 0, 0,
            0, 0, DriveWheels.angularAcceleration,
    });


    /**
     * The left inverse of B matrix to use. Gives the control that results in a target acceleration.
     * @param state     Current state for heading transform.
     * @return          Required Controls
     */
    public Matrix getBLeftInverse(Vector state) {
        return B.transposed().multiplied(1/(DriveWheels.driveAcceleration * DriveWheels.driveAcceleration)).multiplied(h(state).inverted());
    }


    public Matrix h(Vector state) {
        TrigAngle theta = new TrigAngle(state.get(2));
        return h(theta);
    }

    /**
     * Computes heading transform matrix from a precomputed sin-cosine value.
     * @param angle     Trigangle sin-cosine tuple.
     * @return          Heading transformation matrix
     */
    public static Matrix h(TrigAngle angle) {
        return new GeneralMatrix(6, 6, new double[] {
                1, 0, 0, 0, 0, 0,
                0, 1, 0, 0, 0, 0,
                0, 0, 1, 0, 0, 0,
                0, 0, 0, angle.cos, -angle.sin, 0,
                0, 0, 0, angle.sin, angle.cos, 0,
                0, 0, 0, 0, 0, 1,
        });
    }

    /**
     * Derivative of heading transform with respect to an angle.
     * This is technically a tensor, but as few libraries does not support those,
     * this returns the matrix when the index is set to the heading. Everywhere else is zero.
     * @param theta Current heading
     * @return      The derivative.
     */
    public static Matrix dhdtheta(double theta) {
        return dhdtheta(new TrigAngle(theta));
    }

    /**
     * Derivative of heading transform with respect to an angle.
     * This is technically a tensor, but as few libraries does not support those,
     * this returns the matrix when the index is set to the heading. Everywhere else is zero.
     * @param theta Current heading
     * @return      The derivative.
     */
    public static Matrix dhdtheta(TrigAngle theta) {
        return new GeneralMatrix(6, 6, new double[] {
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, -theta.sin, -theta.cos, 0,
                0, 0, 0, theta.cos, -theta.sin, 0,
                0, 0, 0, 0, 0, 0,
        });
    }

    /**
     * Second derivative of heading transform with respect to an angle.
     * This is technically a tensor, but as few libraries does not support those,
     * this returns the matrix when the indices are set to the heading. Everywhere else is zero.
     * @param theta Current heading
     * @return      The derivative.
     */
    public static Matrix dhdthetadtheta(TrigAngle theta) {
        return new GeneralMatrix(6, 6, new double[] {
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, -theta.cos, theta.sin, 0,
                0, 0, 0, -theta.sin, -theta.cos, 0,
                0, 0, 0, 0, 0, 0,
        });
    }

    /**
     * Instantaneous state change model
     * @param x Current state
     * @param u Current control
     * @return  Time derivative of state
     */
    public Vector instantaneousModel(Vector x, Vector u) {

        return h(x).multiplied(linearModel(u, x));

    }

    /**
     * Applies heading transform to A matrix
     * @param state Current state
     * @return  A matrix with heading transformed applied
     */
    public Matrix getAMatrix(Vector state) {
        return h(state).multiplied(A);
    }

    /**
     * Applies heading transform to B matrix
     * @param state Current state
     * @return  B matrix with heading transformed applied
     */
    public Matrix getBMatrix(Vector state) {
        return h(state).multiplied(B);
    }

    /**
     * The linear component of the model.
     * @param state     Current state
     * @param control   Control
     * @return  The derivative without the heading transform applied.
     */
    private Vector linearModel(Vector control, Vector state) {
        Vector acceleration = B.multiplied(controlLimit(control));
        Vector linearModel = Vector.length(6);
        linearModel.add(A.multiplied(state));
        linearModel.add(acceleration);
        return linearModel;
    }

    @Override
    public int getDimensions() {
        return 6;
    }

    @Override
    public int getControls() {
        return 4;
    }

    @Override
    public Vector getLoopback(Vector state) {

        double angle = -state.get(2);
        Matrix h = new GeneralMatrix(3,3, new double[] {
                -Math.sin(angle), Math.cos(angle), 0,
                Math.cos(angle), Math.sin(angle), 0,
                0, 0, 1
        });
        Vector v = h.multiplied(new Vector(state.get(3), state.get(4), 0));
        Vector correction = v.normalized().multiplied(DriveWheels.Lmk);
        correction = correction.multiplied(Math.abs(Math.tanh(DriveWheels.tsv*correction.magnitude())));
        correction.put(1, correction.get(1) * DriveWheels.Lxk);
        correction.put(2, Math.tanh(DriveWheels.tsh*state.get(4)) * DriveWheels.Lhk);
        BaseOpMode.addData("Loopback Drive", correction.get(0));
        BaseOpMode.addData("Loopback Strafe", correction.get(1));
        BaseOpMode.addData("Loopback Turn", correction.get(2));
        return correction;
    }

    @Override
    public Vector toStateSpace(Signal data) {
        return new Vector(new double[] {
                data.getData()[0],
                data.getData()[1],
                data.getData()[2],
                data.getDerivatives()[0],
                data.getDerivatives()[1],
                data.getDerivatives()[2],
        });

    }

    @Override
    public Vector toStateSpace(Vector data, Vector gradient) {
        return new Vector(new double[] {
                data.getData()[0],
                data.getData()[1],
                data.getData()[2],
                gradient.getData()[0],
                gradient.getData()[1],
                gradient.getData()[2],
        });
    }

    /**
     * The control limiting/squashing function. As this is a holonomic drivetrain,
     * Every position in state-space is reachable without worrying about squashing this.
     * @param u     Control
     * @return      Limited Control
     */
    @Override
    public Vector controlLimit(Vector u) {
        u = W.multiplied(u);

        for (int i =0; i < 4; i++) {
            u.put(i, DriveWheels.controlLimit* Math.tanh(u.get(i)));
        }

        return WL.multiplied(u);
    }

    /**
     * Derivative of the control limit function
     * @param control   Current Control
     * @return    The derivative of the limited control with respect to the current control
     */
    @Override
    public Matrix dSdU(Vector control) {
        Matrix sprime = new GeneralMatrix(4,4);
        control = W.multiplied(control);
        sprime.put(0,0, 1/Math.pow(Math.cosh(control.get(0)), 2));
        sprime.put(1,1, 1/Math.pow(Math.cosh(control.get(1)), 2));
        sprime.put(2,2, 1/Math.pow(Math.cosh(control.get(2)), 2));
        sprime.put(3,3, 1/Math.pow(Math.cosh(control.get(3)), 2));

        sprime.multiplied(DriveWheels.controlLimit);

        return WL.multiplied(sprime.multiplied(W));
    }

    /**
     * Estimates the next state given the current state, control, and timestep.
     * Approximates the next state as the current state + derivative * time.
     * @param currentState  Current State
     * @param control       Current Control
     * @param deltatime     Timestep
     * @return              Approximate next state.
     */
    @Override
    public Vector stateTransitionFunction(Vector currentState, Vector control, double deltatime) {

        return currentState.added(h(currentState).multiplied(linearModel(control, currentState)).multiplied(deltatime));
    }

    /**
     * The Derivative of the state transition function with respect to the state
     * @param state     Current state
     * @param control   Current Control
     * @param deltaTime        Timestep
     * @return  The derivative as a Jacobian.
     */
    @Override
    public Matrix dFdX(Vector state, Vector control, double deltaTime) {
        Matrix result = new GeneralMatrix(6, 6);
        TrigAngle angle = new TrigAngle(state.get(2));
        result.add(h(angle).multiplied(A));


        Vector linearModel = dhdtheta(angle).multiplied(linearModel(control,state));

        // Row = a, Col = j = 2
        for (int a = 0; a < 6; a++) {
            result.add(a, 2, linearModel.get(a));
        }

        result.multiply(deltaTime);
        result.add(Matrix.identityMatrix(6));

        return result;
    }

    /**
     * Derivative of the state transition function with respect to the control.
     * @param state      Current state
     * @param control   Current Control
     * @param deltaTime  Timestep
     * @return  The derivative as a Jacobian
     */
    @Override
    public Matrix dFdU(Vector state, Vector control, double deltaTime) {
        return h(state).multiplied(B.multiplied(dSdU(control))).multiplied(deltaTime);
    }

    /**
     * Performs the Matrix-Tensor product with the second derivative of F with respect to x and u.
     * Unfortunately, I didn't have a tensor library, so it works weirdly. This function sums over the
     * raised index of the second derivative when expressed in Einstein index notation, and returns
     * a jxk matrix, where j is the index for the x-derivative and k is the index for the u-derivative.
     *
     * @param state     Current state of the system
     * @param V         the Vector to multiply with
     * @param deltaTime timestep
     * @return The result of the operation
     */
    @Override
    public Matrix VdF2dXdU(Vector state, Vector control, Vector V, double deltaTime) {
        Matrix result = new GeneralMatrix(4, 6);

        Vector col2 = dhdtheta(state.get(2)).multiplied(B.multiplied(dSdU(control))).transposed().multiplied(V);

        for (int k = 0; k < 3; k++) {
                result.put(k, 2, col2.get(k));
        }
        result.multiply(deltaTime);
        return result;
    }

    private double tanh2ndDerivative(double x) {
        return -2*Math.tanh(x)*Math.pow(1/Math.cosh(x), 2);
    }

    /**
     * Performs the Matrix-Tensor product with the second derivative of F with respect to u.
     * Unfortunately, I didn't have a tensor library, so it works weirdly. This function sums over the
     * raised index of the second derivative when expressed in Einstein index notation, and returns
     * a kxk matrix, where k is the index for the u-derivative.
     *
     * @param state     Current state of the system
     * @param vx        the Vector to multiply with
     * @param control   Control
     * @param dt        timestep
     * @return The result of the operation
     */
    @Override
    public Matrix VdF2dUdU(Vector state, Vector control, Vector vx, double dt) {
        Matrix s1D = new GeneralMatrix(4,4, new double[] {
                tanh2ndDerivative(control.get(0)), 0, 0, 0,
                0, tanh2ndDerivative(control.get(1)), 0, 0,
                0, 0, tanh2ndDerivative(control.get(2)), 0,
                0, 0, 0, tanh2ndDerivative(control.get(3)),
        }).multiplied(DriveWheels.controlLimit);
        //Todo: This is probably wrong but I need better tensors to fix.
        Matrix result = h(state).multiplied(B).multiplied(WL).multiplied(s1D).multiplied(W);
        return result;
    }

    /**
     * Performs the Matrix-Tensor product with the second derivative of F with respect to x.
     * Unfortunately, I didn't have a tensor library, so it works weirdly. This function sums over the
     * raised index of the second derivative when expressed in Einstein index notation, and returns
     * a jxk matrix.
     *
     * @param state     Current state of the system
     * @param V         the Vector to multiply with
     * @param control   Control
     * @param deltaTime timestep
     * @return The result of the operation
     */
    public Matrix VdF2dXdX(Vector state, Vector control, Vector V, double deltaTime) {
        Matrix result = new GeneralMatrix(6, 6);
        TrigAngle angle = new TrigAngle(state.get(2));

        Vector crossVector = dhdtheta(angle).multiplied(A).multiplied(V);

        for (int j = 0; j < 6; j++) {
            result.put(j, 2, result.get(j, 2) + crossVector.get(2));
            result.put(2, j, result.get(2, j) + crossVector.get(2));
        }

        Vector linearModel = dhdthetadtheta(angle).multiplied(linearModel(control, state));

        result.put(2, 2, result.get(2,2) + linearModel.dotProduct(V));
        result.multiply(deltaTime);

        return result;
    }

}

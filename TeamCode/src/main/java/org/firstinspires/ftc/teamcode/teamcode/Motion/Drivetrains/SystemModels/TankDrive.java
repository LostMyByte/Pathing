// Primary Author: Kieran Mattingly
package org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains.SystemModels;

import org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.DriveWheels;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.GeneralMatrix;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Matrix;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.TrigAngle;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;

import java.lang.annotation.Documented;

public class TankDrive implements SystemModel{
    // Uses a state-space of [x, y, h, v, vh]
    // Model is xdot = h(x)(Ax + Bu), where
    //      xdot is the derivative of state with respect to time,
    //      x is the state,
    //      u is the control input

    // Derivative of state due to current state.
    private Matrix A = new GeneralMatrix(5, 5, new double[] {
            // The velocity is the derivative of position.
            // Assumes when h=0, velocity is in y direction. This is then fixed by the heading transform.
            0 ,0, 0, 0, 0,
            0 ,0, 0, 1, 0,
            0 ,0, 0, 0, 1,

            // The acceleration is drag forces due to backEMF.
            // Friction is handled separately.
            0 ,0, 0, DriveWheels.Ed, 0,
            0 ,0, 0, 0, DriveWheels.Eh,
    });

    // Control response matrix
    private Matrix B = new GeneralMatrix(5, 2, new double[] {
            0 ,0,
            0 ,0,
            0 ,0,
            // A control affects the PWM duty cycle, which is directly proportional to the torque by the motor.
            // Given that the mass and moment of inertia are (for the most part) constant, the acceleration is
            // directly proportional to motor powers. These values are the scalars that form that proportionality.
            DriveWheels.driveAcceleration/2, DriveWheels.driveAcceleration/2,
            -DriveWheels.angularAcceleration/2, DriveWheels.angularAcceleration/2,
    });

    /**
     * The left inverse of B matrix to use. Gives the control that results in a target acceleration.
     * @param state     Current state for heading transform.
     * @return          Required Controls
     */
    public static Matrix getBLeftInverse(Vector state) {
        return new GeneralMatrix(2, 5, new double[] {
                0, 0, 0, 1/ DriveWheels.driveAcceleration, -1/ DriveWheels.angularAcceleration,
                0, 0, 0, 1/ DriveWheels.driveAcceleration, 1/ DriveWheels.angularAcceleration,
        }).multiplied(hinv(state));
    }

    /**
     * Gets the velocity of each wheel given a robot velocity. Useful for friction calculations.
     * @return The velocity of the wheels as a vector.
      */
    public static Matrix getWheelVelocity() {
        return new GeneralMatrix(2, 5, new double[] {
                0, 0, 0, 1, -DriveWheels.LeverArm,
                0, 0, 0, 1, DriveWheels.LeverArm,
        });
    }


    @Override
    public Matrix h(Vector state) {
        TrigAngle theta = new TrigAngle(state.get(2));
        return h(theta);
    }

    /**
     * Computes heading transform matrix from a precomputed sin-cosine value.
     * @param angle     Trigangle sin-cosine tuple.
     * @return          Heading transformation matrix
     */
    public Matrix h(TrigAngle angle) {
        return new GeneralMatrix(5, 5, new double[] {
                angle.cos, -angle.sin, 0, 0, 0,
                angle.sin, angle.cos, 0, 0, 0,
                0, 0, 1, 0, 0,
                0, 0, 0, 1, 0,
                0, 0, 0, 0, 1,
        });
    }

    /**
     * Inverse of heading transform
     * @param state Current state
     * @return      Heading Transform Inverse
     */
    public static Matrix hinv(Vector state) {
        TrigAngle theta = new TrigAngle(state.get(2));
        return hinv(theta);
    }

    /**
     * Inverse of heading transform
     * @param angle Current angle
     * @return      Heading Transform Inverse
     */
    public static Matrix hinv(TrigAngle angle) {
        return new GeneralMatrix(5, 5, new double[] {
                angle.cos, angle.sin, 0, 0, 0,
                -angle.sin, angle.cos, 0, 0, 0,
                0, 0, 1, 0, 0,
                0, 0, 0, 1, 0,
                0, 0, 0, 0, 1,
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
        return new GeneralMatrix(5, 5, new double[] {
                -theta.sin, -theta.cos, 0, 0, 0,
                theta.cos, -theta.sin, 0, 0, 0,
                0, 0, 0, 0, 0,
                0, 0, 0, 0, 0,
                0, 0, 0, 0, 0,
        });
    }

    /**
     * Second derivative of heading transform with respect to an angle.
     * This is technically a tensor, but as few libraries does not support those,
     * this returns the matrix when the indices are set to the heading. Everywhere else is zero.
     * @param theta Current heading
     * @return      The derivative.
     */
    public Matrix dhdthetadtheta(TrigAngle theta) {
        return new GeneralMatrix(5, 5, new double[] {
                -theta.cos, theta.sin, 0, 0, 0,
                -theta.sin, -theta.cos, 0, 0, 0,
                0, 0, 0, 0, 0,
                0, 0, 0, 0, 0,
                0, 0, 0, 0, 0,
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
        Vector linearModel = Vector.length(5);
        linearModel.add(A.multiplied(state));
        linearModel.add(acceleration);
        return linearModel;
    }

    /**
     * The control limiting/squashing function
     * @param u     Control
     * @return      Limited Control
     */
    @Override
    public Vector controlLimit(Vector u) {
        Vector result = new Vector(2/(Math.exp(-u.get(0)) + 1) - 1, 2/(Math.exp(-u.get(1)) + 1) - 1);
        result.multiply(DriveWheels.controlLimit);
        return result;
    }


    /**
     * Derivative of the control limit function
     * @param u   Current Control
     * @return    The derivative of the limited control with respect to the current control
     */
    @Override
    public Matrix dSdU(Vector u) {
        GeneralMatrix result = new GeneralMatrix(2, 2);

        double cl = DriveWheels.controlLimit;
        for (int index = 0; index < 2; index ++ ) {
            result.put(index, index,  Math.exp(-u.get(index))/(Math.exp(-u.get(index)) + 1));
        }

        result.multiply(cl);
        return result;
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
        Matrix result = new GeneralMatrix(5, 5);
        TrigAngle angle = new TrigAngle(state.get(2));
        result.add(h(angle).multiplied(A));


        Vector linearModel = dhdtheta(angle).multiplied(linearModel(control,state));

        // Row = a, Col = j = 2
        for (int a = 0; a < 5; a++) {
            result.put(a, 2, result.get(a, 2) + linearModel.get(a));
        }

        result.multiply(deltaTime);
        result.add(Matrix.identityMatrix(5));

        return result;
    }

    /**
     * Derivative of the state transition function with respect to the control.
     * @param state      Current state
     * @param _control   Current Control
     * @param deltaTime  Timestep
     * @return  The derivative as a Jacobian
     */
    @Override
    public Matrix dFdU(Vector state, Vector _control, double deltaTime) {
        return h(state).multiplied(B).multiplied(deltaTime);
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
        Matrix result = new GeneralMatrix(2, 5);

        Vector col2 = dhdtheta(state.get(2)).multiplied(B).multiplied(dSdU(control)).transposed().multiplied(V);

        for (int k = 0; k < 2; k++) {
            result.put(k, 2, col2.get(k));
        }
        result.multiply(deltaTime);
        return result;
    }

    /**
     * Performs the Matrix-Tensor product with the second derivative of F with respect to x.
     * Unfortunately, I didn't have a tensor library, so it works weirdly. This function sums over the
     * raised index of the second derivative when expressed in Einstein index notation, and returns
     * a jxj matrix, where j is the index for the x-derivative.
     *
     * @param state     Current state of the system
     * @param V         the Vector to multiply with
     * @param control   Control
     * @param deltaTime timestep
     * @return The result of the operation
     */
    @Override
    public Matrix VdF2dXdX(Vector state, Vector control, Vector V, double deltaTime) {
        Matrix result = new GeneralMatrix(5, 5);
        TrigAngle angle = new TrigAngle(state.get(2));

        Vector crossVector = dhdtheta(angle).multiplied(A).multiplied(V);

        for (int j = 0; j < 5; j++) {
            result.put(j, 2, result.get(j, 2) + crossVector.get(j));
            result.put(2, j, result.get(2, j) + crossVector.get(j));
        }

        Vector linearModel = dhdthetadtheta(angle).multiplied(linearModel(control, state));

        result.put(2, 2, result.get(2,2) + linearModel.dotProduct(V));
        result.multiply(deltaTime);

        return result;
    }

    /**
     * Performs the Matrix-Tensor product with the second derivative of F with respect to u.
     * Unfortunately, I didn't have a tensor library, so it works weirdly. This function sums over the
     * raised index of the second derivative when expressed in Einstein index notation, and returns
     * a kxk matrix, where k is the index for the u-derivative.
     *
     * @param state     Current state of the system
     * @param V         the Vector to multiply with
     * @param control   Control
     * @param deltaTime timestep
     * @return The result of the operation
     */
    @Override
    public Matrix VdF2dUdU(Vector state, Vector control, Vector V, double deltaTime) {
        Matrix i1 = new GeneralMatrix(2, 2);
        Matrix i2 = new GeneralMatrix(2, 2);
        i1.put(0, 0, -2*Math.tanh(control.get(0))/(Math.pow(Math.cosh(control.get(0)), 2)));
        i2.put(1, 1, -2*Math.tanh(control.get(1))/(Math.pow(Math.cosh(control.get(1)), 2)));

        TrigAngle angle = new TrigAngle(state.get(2));

        Matrix col1 = h(angle).multiplied(B).multiplied(i1);
        Vector col2 = h(angle).multiplied(B).multiplied(i2).multiplied(V);

        Matrix result = new GeneralMatrix(2,2);

        for (int i =0; i < 2; i++) {
            result.put(0, i, col1.get(i, i));
            result.put(1, i, col2.get(i));
        }

        result.multiply(deltaTime);

        return result;
    }

    /**
     * Computes the loopback control to handle friction
     * @param pos   Current position in state space
     * @return      Loopback correction in control space
     */
    public static Vector getLoopback(Vector pos) {
        Vector wheelVelocities = TankDrive.getWheelVelocity().multiplied(pos);
        Vector frictionCorrection = new Vector(0,0);

        frictionCorrection.put(0, DriveWheels.Lhk * Math.tanh(DriveWheels.tsh * pos.get(4)));
        frictionCorrection.put(1, -DriveWheels.Lhk * Math.tanh(DriveWheels.tsh * pos.get(4)));

        frictionCorrection.add(0, Math.tanh(DriveWheels.tsv * wheelVelocities.get(0)) * DriveWheels.Lml);
        frictionCorrection.add(1, Math.tanh(DriveWheels.tsv * wheelVelocities.get(1)) * DriveWheels.Lmr);


        return frictionCorrection;
    }
}

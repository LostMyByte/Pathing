package org.firstinspires.ftc.teamcode.Motion;


import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Utilities.Configuration.DriveConfig;
import org.firstinspires.ftc.teamcode.Utilities.Math.GeneralMatrix;
import org.firstinspires.ftc.teamcode.Utilities.Math.Matrix;
import org.firstinspires.ftc.teamcode.Utilities.Math.TrigAngle;
import org.firstinspires.ftc.teamcode.Utilities.Math.Vector;

@Config
public class DriveModel {

    // BackEMF constants
    public static double Ex = 0;
    public static double Ey = 0;
    public static double Eh = 0;

    // Loopback constants
    public static double Lx = 0;
    public static double Ly = 0;
    public static double Lh = 0;

    private static Matrix A = new GeneralMatrix(6, 6, new double[] {
            0 ,0, 0, 1, 0, 0,
            0 ,0, 0, 0, 1, 0,
            0 ,0, 0, 0, 0, 1,
            0 ,0, 0, Ex, 0, 0,
            0 ,0, 0, 1, Ey, 0,
            0 ,0, 0, 1, 0, Eh,
    });

    private static Matrix B = new GeneralMatrix(6, 4, new double[] {
            0 ,0, 0, 0,
            0 ,0, 0, 0,
            0 ,0, 0, 0,
            DriveConfig.DriveWheels.BR.x, DriveConfig.DriveWheels.FR.x, DriveConfig.DriveWheels.BL.x, DriveConfig.DriveWheels.FL.x,
            DriveConfig.DriveWheels.BR.y, DriveConfig.DriveWheels.FR.y, DriveConfig.DriveWheels.BL.y, DriveConfig.DriveWheels.FL.y,
            DriveConfig.DriveWheels.BR.h, DriveConfig.DriveWheels.FR.h, DriveConfig.DriveWheels.BL.h, DriveConfig.DriveWheels.FL.h,
    }).multiplied(DriveConfig.DriveWheels.driveAcceleration);

    private static Vector Ff = new Vector(new double[] {
            0,
            0,
            0,
            Lx,
            Ly,
            Lh
    });



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

    public static Matrix h(Vector state) {
        TrigAngle theta = new TrigAngle(state.get(2));
        return h(theta);
    }

    public static Matrix dhdtheta(double theta) {
        return dhdtheta(new TrigAngle(theta));
    }

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

    public static Vector instantaneousModel(Vector x, Vector u) {
        Vector linearModel = Vector.length(6);
        linearModel.add(A.multiplied(x));
        linearModel.add(B.multiplied(u));
        linearModel.add(Ff);
        return h(x).multiplied(linearModel);

    }
    public static Matrix getAMatrix(Vector state) {
        return GeneralMatrix.diagonalMatrix(6, 1);
    }

    public static Matrix getBMatrix(Vector state) {
        return new GeneralMatrix(6, 3);
    }

    private static Vector linearModel(Vector control, Vector state) {
        Vector linearModel = Vector.length(6);
        linearModel.add(A.multiplied(state));
        linearModel.add(B.multiplied(control));
        linearModel.add(Ff);

        return linearModel;
    }

    // Row = a, Col = j
    public static Matrix dFdX(Vector state, Vector control, double deltaTime) {
        Matrix result = new GeneralMatrix(6, 6);
        TrigAngle angle = new TrigAngle(state.get(2));
        result.add(h(angle).multiplied(A));


        Vector linearModel = dhdtheta(angle).multiplied(linearModel(control,state));

        for (int a = 0; a < 6; a++) {
            result.put(a, 2, result.get(a, 2) + linearModel.get(a));
        }

        result.multiply(deltaTime);
        result.add(Matrix.identityMatrix(6));

        return result;
    }

    public static Vector stateTransitionFunction(Vector currentState, Vector control, double deltatime) {
        return h(currentState).multiplied(linearModel(control, currentState)).multiplied(deltatime);
    }

    public static Matrix dFdU(Vector state, Vector _control, double deltaTime) {
        return h(state).multiplied(B).multiplied(deltaTime);
    }

    /**
     * Performs the Matrix-Tensor product with the second derivative of F with respect to x and u.
     * Unfortunately, I didn't have a tensor library, so it works weirdly. This function sums over the
     * raised index of the second derivative when expressed in Einstein index notation, and returns
     * a jxk matrix, where j is the index for the x-derivative and k is the index for the u-derivative.
     * @param state Current state of the system
     * @param V the Vector to multiply with
     * @param deltaTime timestep
     * @return The result of the operation
     */
    public static Matrix VdF2dXdU(Vector state, Vector V, double deltaTime) {
        Matrix result = new GeneralMatrix(6, 6);

        Vector row2 = dhdtheta(state.get(2)).multiplied(B).multiplied(V);

        for (int k = 0; k < 6; k++) {
                result.put(2, k, row2.get(k));
        }
        result.multiply(deltaTime);
        return result;
    }

    /**
     * Performs the Matrix-Tensor product with the second derivative of F with respect to x.
     * Unfortunately, I didn't have a tensor library, so it works weirdly. This function sums over the
     * raised index of the second derivative when expressed in Einstein index notation, and returns
     * a jxk matrix.
     * @param state Current state of the system
     * @param V the Vector to multiply with
     * @param control Control
     * @param deltaTime timestep
     * @return The result of the operation
     */
    public static Matrix VdF2dXdX(Vector state, Vector control, Vector V, double deltaTime) {
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

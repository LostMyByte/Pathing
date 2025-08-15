package org.firstinspires.ftc.teamcode.Motion.Controllers;

import org.firstinspires.ftc.teamcode.Motion.DriveModel;
import org.firstinspires.ftc.teamcode.Utilities.Math.GeneralMatrix;
import org.firstinspires.ftc.teamcode.Utilities.Math.Matrix;
import org.firstinspires.ftc.teamcode.Utilities.Math.Vector;
import org.ejml.simple.SimpleMatrix;

public class MPC extends Controller {


    public Matrix Q; // Distance cost
    public Matrix R; // Control cost
    public Matrix QF; // Terminal Distance Cost

    public Vector[] currentControls;
    public Vector[] currentTrajectory;

    private int N;



    public MPC(ReferenceSignal referenceSignal, Signal dataSignal, Vector start, Matrix Q, Matrix R, Matrix QF, int N, double threshold) {
        super(referenceSignal, dataSignal);

        this.Q  = Q;
        this.R  = R;
        this.QF = QF;

        this.N = N;

        initializeControls(N);
        generateTrajectory(start);

        double oldcost = getTotalCost();
        updateControls(start);
        double cost = getTotalCost();

        while (oldcost - cost > threshold) {
            oldcost = cost;
            updateControls(start);
            cost = getTotalCost();
        }
    }

    public void generateTrajectory(Vector startState) {

        this.currentTrajectory = new Vector[N];
        Vector currentState = new Vector(startState.getData());

        for (int i = 0; i < N; i++) {
            currentTrajectory[i] = currentState;

            // Simulate Linearized dynamics
            currentState = DriveModel.stateTransitionFunction(currentState, currentControls[i], Signal.deltaTime);
        }
    }

    public double getTotalCost() {
        double currentCost = 0;

        for (int i =0; i < currentControls.length; i++) {
            // Add to cost function
            // TODO: Handle non-constant reference signals
            currentCost += costFunction(currentTrajectory[i], referenceSignal.target(), currentControls[i]);
        }

        // Add terminal cost
        Vector finalState = currentTrajectory[currentTrajectory.length-1];
        currentCost += finalState.dotProduct(QF.multiplied(finalState));

        return currentCost;
    }

    private double costFunction(Vector state, Vector target, Vector control) {
        Vector error = target.subtracted(state);
        return error.dotProduct(Q.multiplied(error)) + control.dotProduct(R.multiplied(control));
    }

    // Derivative of cost function with respect to X
    private Vector dCdX(Vector x) {
        return Q.multiplied(x).multiplied(-2);
    }

    // Derivative of cost function with respect to X when X is final
    private Vector dCFdX(Vector x) {
        return QF.multiplied(x).multiplied(-2);
    }

    // Derivative of cost function with respect to U
    private Vector dCdU(Vector u) {
        return R.multiplied(u).multiplied(2);
    }

    // Second derivative of cost function with respect to X
    private Matrix dCdX2() {
        return Q.multiplied(-2);
    }

    // Second derivative of cost function with respect to X when X is final
    private Matrix dCFdX2() {
        return QF.multiplied(-2);
    }

    // Second derivative of cost function with respect to U
    private Matrix dCdU2() {
        return R.multiplied(2);
    }



    @Override
    public Vector getCorrection() {
        for (int i = 0; i < N-1; i++) {
            currentControls[i] = currentControls[i+1];
            currentTrajectory[i] = currentTrajectory[i+1];
        }

        currentTrajectory[N-1] = DriveModel.stateTransitionFunction(currentTrajectory[N-2], currentControls[N-2], Signal.deltaTime);

        int numDim = this.sensorSignal.getLength();
        Vector sensorData = Vector.length(numDim * 2);

        for (int i = 0; i < numDim; i++) {
            sensorData.put(i, sensorSignal.getIntegralVector().get(i));
            sensorData.put(i+numDim, sensorSignal.getDataVector().get(i));
        }

        updateControls(sensorData);

        return currentControls[0];
    }


    private void initializeControls(int N) {
        this.currentControls = new Vector[N];
        for (int i = 0; i < N; i++) {
            currentControls[i] = new Vector(dimensions);
        }
    }

    private void updateControls(Vector currentState) {

        double dt = Signal.deltaTime;

        Matrix vxx = dCFdX2();
        Vector vx = dCFdX(currentTrajectory[N-1]);
        Vector[] k = new Vector[N];
        Matrix[] K = new Matrix[N];

        for (int i = N-2; i >= 0; i--) {
            Vector control = currentControls[i];
            Vector state = currentTrajectory[i];

            Matrix dfdx = DriveModel.dFdX(state, control, dt);
            Matrix dfdu = DriveModel.dFdU(state, control, dt);

            Vector Qx = dCdX(state).added(dfdx.multiplied(vx));
            Vector Qu = dCdU(control).added(dfdu.multiplied(vx));
            Matrix Qxx = dCdX2().added(dfdx.transposed().multiplied(vxx).multiplied(dfdx)).added(DriveModel.VdF2dXdX(state, control, vx, dt));
            Matrix Qux = dfdu.transposed().multiplied(vxx).multiplied(dfdx).added(DriveModel.VdF2dXdU(state, vx, dt));
            GeneralMatrix Quu = (GeneralMatrix) dCdU2().added(dfdu.transposed().multiplied(vxx).multiplied(dfdu));

            SimpleMatrix Quu2 = new SimpleMatrix(Quu.getData());
            Quu2.reshape(6, 6);
            Quu2 = Quu2.invert();

            Matrix Quu_inv = Matrix.fromEJML(Quu2);

            k[i] = Quu_inv.multiplied(Qu).multiplied(-1);
            K[i] = Quu_inv.multiplied(Qux).multiplied(-1);

            vx = Qx.subtracted(K[i].transposed().multiplied(Quu).multiplied(k[i]));
            vxx = Qxx.subtracted(K[i].transposed().multiplied(Quu).multiplied(K[i]));
        }

        Vector oldstate = currentTrajectory[0];
        currentTrajectory[0] = currentState;

        for (int i = 0; i < N; i++) {
            currentControls[i].add(k[i]);
            currentControls[i].add(K[i].multiplied(currentTrajectory[i].subtracted(oldstate)));

            if (i != N-1) {
                oldstate = currentTrajectory[i + 1];
                currentTrajectory[i + 1] = DriveModel.stateTransitionFunction(currentTrajectory[i], currentControls[i], Signal.deltaTime);

                // TODO: Make this better at not-drivetrains
                // See if it has gone past target
                Vector targetPosition = new Vector(referenceSignal.target().get(0), referenceSignal.target().get(1));
                Vector positionError = targetPosition.subtracted(new Vector(currentTrajectory[i].get(0), currentTrajectory[i].get(1)));
                Vector nextPositionError = targetPosition.subtracted(new Vector(currentTrajectory[i+1].get(0), currentTrajectory[i+1].get(1)));

                if (Math.signum(positionError.dotProduct(nextPositionError)) <=0) {
                    N = i +1;
                }
            }
        }

    }


}

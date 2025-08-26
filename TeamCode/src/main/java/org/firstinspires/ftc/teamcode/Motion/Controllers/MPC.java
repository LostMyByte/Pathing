package org.firstinspires.ftc.teamcode.Motion.Controllers;

import com.acmerobotics.dashboard.config.Config;

import org.ejml.data.DMatrixRMaj;
import org.ejml.data.SingularMatrixException;
import org.ejml.dense.row.EigenOps_DDRM;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.interfaces.decomposition.EigenDecomposition_F64;
import org.ejml.simple.SimpleEVD;
import org.firstinspires.ftc.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.Motion.DriveModel;
import org.firstinspires.ftc.teamcode.Utilities.Math.GeneralMatrix;
import org.firstinspires.ftc.teamcode.Utilities.Math.Matrix;
import org.firstinspires.ftc.teamcode.Utilities.Math.Vector;
import org.ejml.simple.SimpleMatrix;

public class MPC {


    public Matrix Q; // Distance cost
    public Matrix R; // Control cost
    public Matrix QF; // Terminal Distance Cost

    private double lr;
    private double lambda = 1;
    private double lambda_max = 1000;
    private double threshold;

    private double max_vxx;

    public Vector[] currentControls;
    public Vector[] currentTrajectory;

    public ReferenceSignal referenceSignal;
    public Signal sensorSignal;

    private int dimensions;
    private int numControls;

    private int N;


    public MPC(ReferenceSignal referenceSignal, Signal dataSignal, Vector start, Matrix Q, Matrix R, Matrix QF, int N, double threshold, double lr, double lambda_max, double max_vxx) {
        this.referenceSignal = referenceSignal;
        this.sensorSignal = dataSignal;

        this.lambda_max = lambda_max;
        this.dimensions = referenceSignal.getLength();
        this.numControls = 4;

        this.lr = lr;

        this.Q  = Q;
        this.R  = R;
        this.QF = QF;

        this.N = N;
        this.max_vxx = max_vxx;

        initializeControls(N, start);

        this.threshold = threshold;

        iterate(3000, start);

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

        for (int i =0; i < N; i++) {
            // Add to cost function
            // TODO: Handle non-constant reference signals
            currentCost += costFunction(currentTrajectory[i], referenceSignal.target(), currentControls[i], Signal.deltaTime);
        }

        // Add terminal cost
        Vector finalState = currentTrajectory[currentTrajectory.length-1];
        currentCost += finalState.dotProduct(QF.multiplied(finalState));

        return currentCost;
    }

    private double costFunction(Vector state, Vector target, Vector control, double time) {
        Vector error = target.subtracted(state);
        return (error.dotProduct(Q.multiplied(error)) + control.dotProduct(R.multiplied(control)))* time;
    }

    // Derivative of cost function with respect to X
    private Vector dCdX(Vector x, Vector target, double time) {
        return Q.multiplied(target.subtracted(x)).multiplied(-2 * time);
    }

    // Derivative of cost function with respect to X when X is final
    private Vector dCFdX(Vector x, Vector target) {
        return QF.multiplied(target.subtracted(x)).multiplied(-2);
    }

    // Derivative of cost function with respect to U
    private Vector dCdU(Vector u, double time) {
        return R.multiplied(u).multiplied(2*time);
    }

    // Second derivative of cost function with respect to X
    private Matrix dCdX2(double time) {
        return Q.multiplied(2 * time);
    }

    // Second derivative of cost function with respect to X when X is final
    private Matrix dCFdX2() {
        return QF.multiplied(2);
    }

    // Second derivative of cost function with respect to U
    private Matrix dCdU2(double time) {
        return R.multiplied(2 * time);
    }



    public Vector getCorrection() {
        for (int i = 0; i < N-1; i++) {
            currentControls[i] = currentControls[i+1];
            currentTrajectory[i] = currentTrajectory[i+1];
        }

        if (N > 1) currentTrajectory[N-1] = DriveModel.stateTransitionFunction(currentTrajectory[N-2], currentControls[N-2], Signal.deltaTime);
         

        int numDim = this.sensorSignal.getLength();
        Vector sensorData = Vector.length(numDim * 2);

        for (int i = 0; i < numDim; i++) {
            sensorData.put(i, sensorSignal.getIntegralVector().get(i));
            sensorData.put(i+numDim, sensorSignal.getDataVector().get(i));
        }

        iterate(2, sensorData);

        return currentControls[0];
    }


    private void initializeControls(int N, Vector start) {
        this.currentControls = new Vector[N];
        this.currentTrajectory = new Vector[N];
        this.currentTrajectory[0] = start;
        for (int i = 0; i < N; i++) {
            Vector error = referenceSignal.target().subtracted(currentTrajectory[i]).multiplied(1/((N-i) * Signal.deltaTime));

            currentControls[i] = Vector.withValue(0, numControls);//DriveModel.getBLeftInverse(currentTrajectory[i]).multiplied(error);

            if (i != N-1) {
                currentTrajectory[i+1] = DriveModel.stateTransitionFunction(currentTrajectory[i], currentControls[i], Signal.deltaTime);
            }
        }

    }

    private void updateControls(Vector currentState) {

        double dt = Signal.deltaTime;

        Matrix vxx = dCFdX2();
        Vector vx = dCFdX(currentTrajectory[N-1], referenceSignal.target());
        Vector[] k = new Vector[N];
        Matrix[] K = new Matrix[N];

        for (int i = N-1; i >= 0; i--) {
            Vector control = currentControls[i];
            Vector state = currentTrajectory[i];

            Matrix dfdx = DriveModel.dFdX(state, control, dt);
            Matrix dfdxT = dfdx.transposed();
            Matrix dfdu = DriveModel.dFdU(state, control, dt);
            Matrix dfduT = dfdu.transposed();

            Vector Qx = dCdX(state, referenceSignal.target(), dt).added(dfdxT.multiplied(vx));
            Vector Qu = dCdU(control, dt).added(dfduT.multiplied(vx));
            Matrix Qxx = dCdX2(dt).added(dfdxT.multiplied(vxx).multiplied(dfdx)).added(DriveModel.VdF2dXdX(state, control, vx, dt));
            Matrix Qux = dfduT.multiplied(vxx).multiplied(dfdx).added(DriveModel.VdF2dXdU(state, vx, dt));
            GeneralMatrix Quu = (GeneralMatrix) dCdU2(dt).added(dfduT.multiplied(vxx).multiplied(dfdu));

            DMatrixRMaj Quu2 = new DMatrixRMaj(this.numControls, this.numControls, true, Quu.getData());

            EigenDecomposition_F64<DMatrixRMaj> eigs = DecompositionFactory_DDRM.eig(true);
            eigs.decompose(Quu2);

            DMatrixRMaj EVecs = EigenOps_DDRM.createMatrixV(eigs);
            DMatrixRMaj EVals = EigenOps_DDRM.createMatrixD(eigs);

            for (int eign = 0; eign < EVals.numRows; eign++) {
                if (EVals.get(eign, eign) < 0) EVals.set(eign, eign, 0);
                EVals.add(eign, eign, lambda);
                EVals.set(eign, eign, 1/EVals.get(eign, eign));
            }

            DMatrixRMaj Quu2_inv = new DMatrixRMaj(this.numControls, this.numControls);
            DMatrixRMaj temp = new DMatrixRMaj(this.numControls, this.numControls);
            CommonOps_DDRM.mult(EVecs, EVals, temp);
            CommonOps_DDRM.invert(EVecs);
            CommonOps_DDRM.mult(temp, EVecs, Quu2_inv);

            Matrix Quu_inv = new GeneralMatrix(this.numControls, this.numControls, Quu2_inv.getData());

            k[i] = Quu_inv.multiplied(Qu).multiplied(-1);
            K[i] = Quu_inv.multiplied(Qux).multiplied(-1);

            for (int c = 0; c< numControls; c++) {
                if (Math.abs(k[i].get(c) + control.get(c)) > 1) {
                    k[i].put(c, (1 - Math.abs(control.get(c))) * Math.signum(control.get(c)));
                }
            }

            vx = Qx.subtracted(K[i].transposed().multiplied(k[i]));
            vxx = Qxx.subtracted(K[i].transposed().multiplied(K[i]));

            for (int r = 0; r < dimensions; r++) {
                for (int c = 0; c < dimensions; c++) {
                    if (Math.abs(vxx.get(r, c)) > max_vxx) vxx.put(r, c, max_vxx * Math.signum(vxx.get(r, c)));
                }
            }
        }

        Vector oldstate = currentTrajectory[0];
        currentTrajectory[0] = currentState;

        for (int i = 0; i < N; i++) {
            currentControls[i] = currentControls[i].added(k[i]);
            currentControls[i].add(K[i].multiplied(currentTrajectory[i].subtracted(oldstate)));

            for (int c = 0; c < numControls; c++) {
                if (Math.abs(currentControls[i].get(c)) > 1) currentControls[i].put(c, Math.signum(currentControls[i].get(c)));
            }

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
                    BaseOpMode.addData("Setting N to", N);
                }
            }
        }

        BaseOpMode.addData("Horizon Time", Signal.deltaTime * N);

    }

    private void iterate(int maxIter, Vector start) {

        this.lambda = 1;

        double oldcost = getTotalCost();
        BaseOpMode.addData("MPC: Initial Cost", oldcost);

        Vector[] xold = new Vector[N];
        Vector[] uold = new Vector[N];
        System.arraycopy(currentTrajectory, 0, xold, 0, N);
        System.arraycopy(currentControls, 0, uold, 0, N);

        updateControls(start);
        BaseOpMode.addData("MPC", "Controls updated");
        double cost = getTotalCost();


        for (int iter = 0; (iter < maxIter) || (maxIter == 0); iter++) {
            if (cost < oldcost) {

                BaseOpMode.addData("MPC: Cost", cost);
                BaseOpMode.addData("MPC: Old Cost", oldcost);
                BaseOpMode.addData("MPC: Improvement", oldcost - cost);

                BaseOpMode.addData("MPC: Landing X", currentTrajectory[N-1].get(0));
                BaseOpMode.addData("MPC: Landing Y", currentTrajectory[N-1].get(1));
                BaseOpMode.addData("MPC: Landing H", currentTrajectory[N-1].get(2));

                lambda /= lr;
                System.arraycopy(currentTrajectory, 0, xold, 0, N);
                System.arraycopy(currentControls, 0, uold, 0, N);

                if (Math.abs(cost-oldcost)/cost < threshold) {
                    break;
                }
                oldcost = cost;
            }
            else {
                System.arraycopy(xold, 0, currentTrajectory, 0, N);
                System.arraycopy(uold, 0, currentControls, 0, N);
                lambda *= lr;
                if (lambda > lambda_max) break;
            }


            updateControls(start);
            cost = getTotalCost();


            BaseOpMode.addData("MPC: Iteration", iter);
            BaseOpMode.addData("MPC: lambda", lambda);


            BaseOpMode.updateTelemetry();
        }

    }


}

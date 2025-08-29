package org.firstinspires.ftc.teamcode.Motion.Controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    private Vector[] currentControls;
    public Vector[] k;
    public Matrix[] K;
    public Vector[] currentTrajectory;

    public ReferenceSignal referenceSignal;
    public Signal sensorSignal;

    private int dimensions;
    private int numControls;

    private int N;
    private double horizon;
    private double dt;

    private ElapsedTime timer;


    public MPC(ReferenceSignal referenceSignal, Signal dataSignal, Vector start, Matrix Q, Matrix R, Matrix QF, int N, double time, double threshold, double lr, double lambda_max, double max_vxx) {
        this.referenceSignal = referenceSignal;
        this.sensorSignal = dataSignal;

        this.lambda_max = lambda_max;
        this.dimensions = referenceSignal.getLength();
        this.numControls = 3;
        this.horizon = time;

        this.lr = lr;

        this.Q  = Q;
        this.R  = R;
        this.QF = QF;

        this.N = N;
        this.max_vxx = max_vxx;

        this.dt = horizon/N;
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
            currentState = DriveModel.stateTransitionFunction(currentState, currentControls[i], dt);
        }
    }

    public double getTotalCost() {
        double currentCost = 0;

        for (int i =0; i < N; i++) {
            // Add to cost function
            // TODO: Handle non-constant reference signals
            currentCost += costFunction(currentTrajectory[i], referenceSignal.target(), currentControls[i], dt);
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

    public void start() {
        this.timer = new ElapsedTime();
    }

    public Vector getCorrection() {

        if (timer == null) start();
        double time = timer.time();

        int numDim = this.sensorSignal.getLength();
        Vector sensorData = Vector.length(numDim * 2);

        for (int i = 0; i < numDim; i++) {
            sensorData.put(i, sensorSignal.getIntegralVector().get(i));
            sensorData.put(i+numDim, sensorSignal.getDataVector().get(i));
        }
        Vector target = getInterpolatedX(time);
        BaseOpMode.addData("TX", target.get(0));
        BaseOpMode.addData("TY", target.get(1));
        BaseOpMode.addData("TH", target.get(2));
        BaseOpMode.addData("TVX", target.get(3));
        BaseOpMode.addData("TVY", target.get(4));
        BaseOpMode.addData("TVH", target.get(5));

        sensorData.subtract(target);
        Vector correction = getInterpolatedU(time);
        return correction.added(getInterpolatedK(time).multiplied(sensorData));
    }


    private void initializeControls(int N, Vector start) {
        this.currentControls = new Vector[N];
        this.currentTrajectory = new Vector[N];
        this.currentTrajectory[0] = start;
        this.k = new Vector[N];
        this.K = new Matrix[N];
        for (int i = 0; i < N; i++) {
            Vector error = referenceSignal.target().subtracted(currentTrajectory[i]).multiplied(1/((N-i) * dt));

            currentControls[i] = Vector.withValue(0, numControls);//DriveModel.getBLeftInverse(currentTrajectory[i]).multiplied(error);

            if (i != N-1) {
                currentTrajectory[i+1] = DriveModel.stateTransitionFunction(currentTrajectory[i], currentControls[i], dt);
            }
        }

    }

    private Vector getInterpolatedk(double time) {
        return getInterpolatedk(time, horizon);
    }
    private Vector getInterpolatedU(double time) {
        return getInterpolatedU(time, horizon);
    }
    private Vector getInterpolatedX(double time) {
        return getInterpolatedX(time, horizon);
    }
    private Matrix getInterpolatedK(double time) {
        return getInterpolatedK(time, horizon);
    }

    private Vector getInterpolatedk(double time, double horizon) {
        if (time >= (horizon-dt)) return k[N-1];
        double position = (time/horizon) * N;
        int index = (int) position;
        double alpha = position - index;

        return (k[index].multiplied(1 - alpha).added(k[index + 1].multiplied(alpha)));
    }

    private Matrix getInterpolatedK(double time, double horizon) {
        if (time >= (horizon-dt)) return K[N-1];
        double position = (time/horizon) * N;
        int index = (int) position;
        double alpha = position - index;

        return (K[index].multiplied(1 - alpha).added(K[index + 1].multiplied(alpha)));
    }

    private Vector getInterpolatedX(double time, double horizon) {
        if (time >= (horizon-dt)) return currentTrajectory[N-1];
        double position = (time/horizon) * N;
        int index = (int) position;
        double alpha = position - index;

        return (currentTrajectory[index].multiplied(1 - alpha).added(currentTrajectory[index + 1].multiplied(alpha)));
    }
    private Vector getInterpolatedU(double time, double horizon) {
        if (time >= (horizon-dt)) return currentControls[N-1];
        double position = (time/horizon) * N;
        int index = (int) position;
        double alpha = position - index;

        return (currentControls[index].multiplied(1 - alpha).added(currentControls[index + 1].multiplied(alpha)));
    }

    private void updateControls(Vector currentState) {

        Matrix vxx = dCFdX2();
        Vector vx = dCFdX(currentTrajectory[N-1], referenceSignal.target());
        k = new Vector[N];
        K = new Matrix[N];

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
                if (EVals.get(eign, eign) < 0) {
                    EVals.set(eign, eign, 0);
                }
                EVals.add(eign, eign, lambda);
                EVals.set(eign, eign, 1/EVals.get(eign, eign));
            }

            DMatrixRMaj Quu2_inv = new DMatrixRMaj(this.numControls, this.numControls);
            DMatrixRMaj temp = new DMatrixRMaj(this.numControls, this.numControls);
            CommonOps_DDRM.mult(EVecs, EVals, temp);
            CommonOps_DDRM.invert(EVecs);
            CommonOps_DDRM.mult(temp, EVecs, Quu2_inv);

            //CommonOps_DDRM.invert(Quu2, Quu2_inv);


            Matrix Quu_inv = new GeneralMatrix(this.numControls, this.numControls, Quu2_inv.getData());

            k[i] = Quu_inv.multiplied(Qu).multiplied(-1);
            K[i] = Quu_inv.multiplied(Qux).multiplied(-1);

            /*for (int c = 0; c< numControls; c++) {
                if (Math.abs(k[i].get(c) + control.get(c)) > 1) {
                    k[i].put(c, (1 - Math.abs(control.get(c))) * Math.signum(control.get(c)));
                }
            }*/

            vx = Qx.subtracted(K[i].transposed().multiplied(k[i]));
            vxx = Qxx.subtracted(K[i].transposed().multiplied(K[i]));

            /*for (int r = 0; r < dimensions; r++) {
                for (int c = 0; c < dimensions; c++) {
                    if (Math.abs(vxx.get(r, c)) > max_vxx) {
                        vxx.put(r, c, max_vxx * Math.signum(vxx.get(r, c)));
                    }
                }
            }*/
        }

        Vector oldstate = currentTrajectory[0];
        currentTrajectory[0] = currentState;

        for (int i = 0; i < N; i++) {
            currentControls[i] = currentControls[i].added(k[i]);
            currentControls[i].add(K[i].multiplied(currentTrajectory[i].subtracted(oldstate)));

            if (i != N-1) {
                oldstate = currentTrajectory[i + 1];
                currentTrajectory[i + 1] = DriveModel.stateTransitionFunction(currentTrajectory[i], currentControls[i], dt);

                // TODO: Make this better at not-drivetrains
                // See if it has gone past target
                Vector targetPosition = new Vector(referenceSignal.target().get(0), referenceSignal.target().get(1));
                Vector positionError = targetPosition.subtracted(new Vector(currentTrajectory[i].get(0), currentTrajectory[i].get(1)));
                Vector nextPositionError = targetPosition.subtracted(new Vector(currentTrajectory[i+1].get(0), currentTrajectory[i+1].get(1)));

                if (Math.signum(positionError.dotProduct(nextPositionError)) <=0 && (referenceSignal.target().get(3) != 0 || referenceSignal.target().get(4) != 0 || referenceSignal.target().get(5) != 0)) {
                    double newhorizon = (i+1)*dt;
                    double newdt = newhorizon/N;
                    BaseOpMode.addData("Setting horizon to", newhorizon);

                    for (int j = 0; j < N; j++) {
                        double t = newdt * j;
                        currentControls[j] = getInterpolatedU(t, newhorizon);
                        currentTrajectory[j] = getInterpolatedX(t, newhorizon);
                        k[j] = getInterpolatedk(t, newhorizon);
                        K[j] = getInterpolatedK(t, newhorizon);
                    }

                    this.dt = newdt;
                    this.horizon = newhorizon;
                }
            }
        }


    }

    private void iterate(int maxIter, Vector start) {

        this.lambda = 1;

        double oldcost = getTotalCost();
        BaseOpMode.addData("MPC: Initial Cost", oldcost);

        Vector[] xold = currentTrajectory.clone();
        Vector[] uold = currentControls.clone();
        Vector[] kold = k.clone();
        Matrix[] Kold = K.clone();

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
                xold = currentTrajectory.clone();
                uold = currentControls.clone();
                kold = k.clone();
                Kold = K.clone();

                if (Math.abs(cost-oldcost)/cost < threshold) {
                    break;
                }
                oldcost = cost;
            }
            else {
                lambda *= lr;

                currentTrajectory = xold.clone();
                currentControls = uold.clone();
                k = kold.clone();
                K = Kold.clone();
                if (lambda > lambda_max) {
                    break;
                }
            }


            updateControls(start);
            cost = getTotalCost();


            BaseOpMode.addData("MPC: Iteration", iter);
            BaseOpMode.addData("MPC: lambda", lambda);


            BaseOpMode.updateTelemetry();
        }

    }


}

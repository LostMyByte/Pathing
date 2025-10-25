package org.firstinspires.ftc.teamcode.teamcode.Motion.Controllers;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.EigenOps_DDRM;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.interfaces.decomposition.EigenDecomposition_F64;
import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;

import org.firstinspires.ftc.teamcode.teamcode.Motion.SystemModels.SystemModel;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.GeneralMatrix;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Matrix;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;

public class MPC {

    public Matrix Q; // Distance cost
    public Matrix R; // Control cost
    public Matrix QF; // Terminal Distance Cost

    private double lr;
    private double lambda = 1;
    private double lambda_max = 1000;
    private double threshold;


    public Vector[] currentControls;
    public Vector[] k;
    public Matrix[] K;
    public Vector[] currentTrajectory;

    public ReferenceSignal referenceSignal;
    public Signal sensorSignal;

    public int dimensions;
    public int numControls;

    public int N;
    private double horizon;
    private double dt;

    private ElapsedTime timer;
    
    private SystemModel model;


    public MPC(ReferenceSignal referenceSignal, Signal dataSignal, Vector start, Matrix Q, Matrix R, Matrix QF, int N, double time, double threshold, double lr, double lambda_max, SystemModel model) {
        this.referenceSignal = referenceSignal;
        this.sensorSignal = dataSignal;

        this.lambda_max = lambda_max;
        this.dimensions = referenceSignal.getLength();
        this.numControls = 2;
        this.horizon = time;

        this.lr = lr;

        this.Q  = Q;
        this.R  = R;
        this.QF = QF;

        this.N = N;

        this.model = model;

        this.dt = horizon/N;
        initializeControls(N, start);

        this.threshold = threshold;

        this.lambda = 1;


    }



    public void generateTrajectory(Vector startState) {

        this.currentTrajectory = new Vector[N];
        Vector currentState = new Vector(startState.getData());

        for (int i = 0; i < N; i++) {
            currentTrajectory[i] = currentState;

            // Simulate Linearized dynamics
            currentState = model.stateTransitionFunction(currentState, currentControls[i], dt);
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

    private void initializeControls(int N, Vector start) {
        this.currentControls = new Vector[N];
        this.currentTrajectory = new Vector[N];
        this.currentTrajectory[0] = start;
        this.k = new Vector[N];
        this.K = new Matrix[N];
        for (int i = 0; i < N; i++) {
            Vector error = referenceSignal.target().subtracted(currentTrajectory[i]).multiplied(1/((N-i) * dt));

            currentControls[i] = Vector.withValue(0, numControls);//model.getBLeftInverse(currentTrajectory[i]).multiplied(error);

            this.k[i] = Vector.length(2);
            this.K[i] = new GeneralMatrix(2, 5);
            if (i != N-1) {
                currentTrajectory[i+1] = model.stateTransitionFunction(currentTrajectory[i], currentControls[i], dt);
            }
        }

    }

    public Vector getInterpolatedk(double time) {
        return getInterpolatedk(time, horizon);
    }
    public Vector getInterpolatedU(double time) {
        return getInterpolatedU(time, horizon);
    }
    public Vector getInterpolatedX(double time) {
        return getInterpolatedX(time, horizon);
    }
    public Matrix getInterpolatedK(double time) {
        return getInterpolatedK(time, horizon);
    }

    public Vector getInterpolatedk(double time, double horizon) {
        if (time >= (horizon-2 * dt)) return k[N-2];
        double position = (time/horizon) * N;
        int index = (int) position;
        double alpha = position - index;

        return (k[index].multiplied(1 - alpha).added(k[index + 1].multiplied(alpha)));
    }

    public Matrix getInterpolatedK(double time, double horizon) {
        if (time >= (horizon-2* dt)) return K[N-2];
        double position = (time/horizon) * N;
        int index = (int) position;
        double alpha = position - index;

        return (K[index].multiplied(1 - alpha).added(K[index + 1].multiplied(alpha)));
    }

    public Vector getInterpolatedX(double time, double horizon) {
        if (time >= (horizon-dt)) return currentTrajectory[N-1];
        double position = (time/horizon) * N;
        int index = (int) position;
        double alpha = position - index;

        return (currentTrajectory[index].multiplied(1 - alpha).added(currentTrajectory[index + 1].multiplied(alpha)));
    }
    public Vector getInterpolatedU(double time, double horizon) {
        if (time >= (horizon-2*dt)) return Vector.length(numControls);
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

            Matrix dfdx = model.dFdX(state, control, dt);
            Matrix dfdxT = dfdx.transposed();
            Matrix dfdu = model.dFdU(state, control, dt);
            Matrix dfduT = dfdu.transposed();

            Vector Qx = dCdX(state, referenceSignal.target(), dt).added(dfdxT.multiplied(vx));
            Vector Qu = dCdU(control, dt).added(dfduT.multiplied(vx));
            Matrix Qxx = dCdX2(dt).added(dfdxT.multiplied(vxx).multiplied(dfdx));//.added(model.VdF2dXdX(state, control, vx, dt));
            Matrix Qux = dfduT.multiplied(vxx).multiplied(dfdx);//.added(model.VdF2dXdU(state, control, vx, dt));
            GeneralMatrix Quu = (GeneralMatrix) dCdU2(dt).added(dfduT.multiplied(vxx).multiplied(dfdu)).added(model.VdF2dUdU(state,control,vx, dt));

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
                currentTrajectory[i + 1] = model.stateTransitionFunction(currentTrajectory[i], currentControls[i], dt);

                // TODO: Make this better at not-drivetrains
                // See if it has gone past target
                Vector targetPosition = new Vector(referenceSignal.target().get(0), referenceSignal.target().get(1));
                Vector positionError = targetPosition.subtracted(new Vector(currentTrajectory[i].get(0), currentTrajectory[i].get(1)));
                Vector nextPositionError = targetPosition.subtracted(new Vector(currentTrajectory[i+1].get(0), currentTrajectory[i+1].get(1)));

                if (Math.signum(positionError.dotProduct(nextPositionError)) <=0 && (referenceSignal.target().get(3) != 0 || referenceSignal.target().get(4) != 0 )) {
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

    protected void stepForwardHorizon(Vector currentPos, int currentN) {

        if (currentN == 0) return;

        lambda = 125;


        for (int i = 0; i < N - currentN; i++) {
            this.currentControls[i] = this.currentControls[currentN + i];
            this.currentTrajectory[i] = this.currentTrajectory[currentN + i];
            this.k[i] = this.k[currentN + i];
            this.K[i] = this.K[currentN + i];
        }

        for (int i = currentN; i < N; i++) {
            this.k[i] = this.k[i-1];
            this.K[i] = this.K[i-1];
            this.currentControls[i] = currentControls[N-1].added(k[i]);
        }

        Vector oldstate = currentTrajectory[0];
        currentTrajectory[0] = currentPos;

        for (int i = 0; i < N; i++ ) {
            currentControls[i] = currentControls[i].added(k[i]);
            currentControls[i].add(K[i].multiplied(currentTrajectory[i].subtracted(oldstate)));

            if (i != N-1) {
                oldstate = currentTrajectory[i + 1];
                currentTrajectory[i + 1] = model.stateTransitionFunction(currentTrajectory[i], currentControls[i], dt);
            }
        }



    }


    public void loadFromArray(Vector[] x, Vector[] u, Vector[] k, Matrix[] K) {
        this.currentTrajectory = x;
        this.currentControls = u;
        this.k = k;
        this.K = K;
    }

    public void iterate(int maxIter, Vector start) {



        double oldcost = getTotalCost();
        BaseOpMode.addData("MPC: Initial Cost", oldcost);

        Vector[] xold = currentTrajectory.clone();
        Vector[] uold = currentControls.clone();
        Vector[] kold = k.clone();
        Matrix[] Kold = K.clone();

        BaseOpMode.addData("MPC", "Controls updated");
        double cost = getTotalCost();


        for (int iter = 0; (iter < maxIter) || (maxIter == 0); iter++) {
            updateControls(start);
            cost = getTotalCost();


            BaseOpMode.addData("MPC: Iteration", iter);
            BaseOpMode.addData("MPC: lambda", lambda);


            if (cost < oldcost ) {

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

            BaseOpMode.updateTelemetry();

        }

    }


}

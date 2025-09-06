package org.firstinspires.ftc.teamcode.teamcode.Utilities.Learning;

import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode.addData;
import static org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.KCP.Localization.Location;
import org.firstinspires.ftc.teamcode.teamcode.KCP.Localization.TwoWheelOdometry;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Hardware;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Control.PID;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.LinearAlgebra.GeneralMatrix;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.LinearAlgebra.Matrix;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.LinearAlgebra.Vector;

public class PIDTuningDriveTrain extends Learnable {

    private PID pid;

    private DcMotor motorFL, motorBR, motorFR, motorBL;

    private Location odo;

    private double dist;

    public double threshold;

    public double minPos, maxPos;

    private static Matrix rotationMatrix = new GeneralMatrix(2, 2, new double[] {0.7071068, 0.7071068, -0.7071068, 0.7071068});

    public PIDTuningDriveTrain(double min, double max) {
        this.minPos = min;
        this.maxPos = max;

        motorFR = hardware.get(DcMotor.class, Hardware.rightFront);
        motorFL = hardware.get(DcMotor.class, Hardware.leftFront);
        motorBR = hardware.get(DcMotor.class, Hardware.rightBack);
        motorBL = hardware.get(DcMotor.class, Hardware.leftBack);

        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        motorFR.setDirection(DcMotor.Direction.FORWARD);

        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);

        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        motorBR.setDirection(DcMotor.Direction.FORWARD);

        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        motorBL.setDirection(DcMotor.Direction.REVERSE);

        odo = new TwoWheelOdometry(0 ,0, 0);

        pid = new PID(0,0,0);


    }

    public void move(Vector direction) {
        // direction is a 2x1 vector
        Vector out = rotationMatrix.multiplied(direction);

        motorFL.setPower(out.get(0));
        motorBR.setPower(out.get(0));

        motorFR.setPower(out.get(1));
        motorBL.setPower(out.get(1));
    }

    @Override
    public double fitness(Vector parameters) {

        double target = dist > 0 ? dist : 0;
        dist = -dist;
        pid.setConstants(parameters.get(0), parameters.get(1), parameters.get(2));
        double start = System.nanoTime();
        //Parameters will be kP, kI, kD
        // Result will be time to move to the target
        odo.update();
        while (Math.abs(odo.y() - target) > 5) {
            odo.update();
            move(new Vector(0, pid.getCorrection(odo.y(), target)));
            addData("Position", odo.y());
            BaseOpMode.updateTelemetry();

        }
        double end = System.nanoTime();
        return start-end;
    }

    @Override
    public void update() {
        dist = Vector.random(1, minPos, maxPos).get(0);
    }
}

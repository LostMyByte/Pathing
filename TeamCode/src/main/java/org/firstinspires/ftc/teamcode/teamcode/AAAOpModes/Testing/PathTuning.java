// Primary Author: Kieran Mattingly
package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;

import org.firstinspires.ftc.teamcode.teamcode.Motion.Signals.Signal;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains.TankDriveTrain;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains.SystemModels.TankDrive;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;

@TeleOp(name = "PathingTuning")


public class PathTuning extends BaseOpMode {
    private TankDriveTrain drive;

    @Config
    public static class tuningParams {
        public static double A1 = 0;
        public static double A2 = 0;
        public static double AT = 0;
        public static double B1 = 0;
        public static double B2 = 0;
        public static double BT = 0;
        public static double C1 = 0;
        public static double C2 = 0;
        public static double CT = 0;
    }

    ElapsedTime timer;

    Vector oldState;
    TankDrive model;
    @Override
    public void externalInit() {
        this.drive = new TankDriveTrain(new Vector(0, 0, 0));

        model = new TankDrive();
        oldState = drive.loc.getPositionForTankDrive();
        this.timer = new ElapsedTime();
    }

    @Override
    public void externalInitLoop() {
        timer.reset();
        addData("Predicted X", oldState.get(0));
        addData("Predicted Y", oldState.get(1));
        addData("Predicted H", oldState.get(2));
        addData("Predicted V", oldState.get(3));
        addData("Predicted VH", oldState.get(4));

    }
    @Override
    public void externalLoop() {
        double time = timer.time();
        Vector target;
        if (time < tuningParams.AT) {
            target =  new Vector(tuningParams.A1, tuningParams.A2);
        }
        else if (time < tuningParams.BT) {
            target =  new Vector(tuningParams.B1, tuningParams.B2);
        }
        else if (time < tuningParams.CT) {
            target =  new Vector(tuningParams.C1, tuningParams.C2);
        }
        else {
            target = Vector.length(2);
        }
        drive.moveRaw(target);
        drive.loc.getPositionForTankDrive();
        oldState = model.stateTransitionFunction(oldState, target, Signal.deltaTime);
        addData("Predicted X", oldState.get(0));
        addData("Predicted Y", oldState.get(1));
        addData("Predicted H", oldState.get(2));
        addData("Predicted V", oldState.get(3));
        addData("Predicted VH", oldState.get(4));
    }
}

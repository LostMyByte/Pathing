package org.firstinspires.ftc.teamcode.Motion.Drivetrains;

import org.firstinspires.ftc.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.Motion.Movement;
import org.firstinspires.ftc.teamcode.Utilities.Math.Vector;

import org.firstinspires.ftc.teamcode.Utilities.Configuration.DriveConfig;


// A general fixed-wheel Holonomic drivetrain class
public class FixedDriveTrain extends Movement {

    public FixedDriveTrain(Vector startState) {
        super(startState);
    }

    public void move(Vector target) {
        move(target, false);
    }

    // Sets drivetrain power to a target power vector
    public void move(Vector target, boolean useFullPower) {

        double[] powers = new double[driveWheels.length];
        double maxPower = 0;

        target.put(0, target.get(0) + DriveConfig.DriveWheels.Lxk*Math.signum(target.get(0)));

        // Get power of each wheel with dot product
        for (int i = 0; i < driveWheels.length; i++) {
            powers[i] = driveWheels[i].MovementVector.dotProduct(target);

            powers[i] += DriveConfig.DriveWheels.Lmk*Math.signum(driveWheels[i].encoder.getVelocity());

            if (Math.abs(powers[i]) > maxPower) maxPower = Math.abs(powers[i]);
        }

        // Make sure that the speed is capped so that it doesn't go in the wrong direction
        if (maxPower > 1 || useFullPower) {
            for (int i = 0; i < driveWheels.length; i++) {
                powers[i] /= maxPower;
            }
        }

        BaseOpMode.addData("Max Power", maxPower);
        // Command motor powers
        for (int i = 0; i < driveWheels.length; i++) {
            driveWheels[i].setPower(powers[i]);
            BaseOpMode.addData(String.format("Setting Motor %d to Power", i), powers[i]);
        }
    }

    public void move(double drive, double strafe, double turn, double speed) {
        move(new Vector(drive, strafe, turn).multiplied(speed));
    }

    public void moveRaw(Vector powers) {

        for (int i =0; i < driveWheels.length; i++) {
            driveWheels[i].setPower(powers.get(i));
        }
    }
}

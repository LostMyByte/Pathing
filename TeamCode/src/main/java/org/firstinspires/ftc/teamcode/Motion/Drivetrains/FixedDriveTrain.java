package org.firstinspires.ftc.teamcode.Motion.Drivetrains;

import org.firstinspires.ftc.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.Utilities.LinearAlgebra.Vector;

import org.firstinspires.ftc.teamcode.Utilities.Configuration.DriveConfig;


// A general fixed-wheel Holonomic drivetrain class
public class FixedDriveTrain {

    public void move(Vector target) {
        move(target, false);
    }

    // Moves along a target vector
    public void move(Vector target, boolean useFullPower) {

        double[] powers = new double[DriveConfig.driveWheels.length];
        double maxPower = 0;

        // Get power of each wheel with dot product
        for (int i = 0; i < DriveConfig.driveWheels.length; i++) {
            powers[i] = DriveConfig.driveWheels[i].MovementVector.dotProduct(target);

            if (Math.abs(powers[i]) > maxPower) maxPower = Math.abs(powers[i]);
        }

        // Make sure that the speed is capped so that it doesn't go too fast in the wrong direction
        if (maxPower > 1 || useFullPower) {
            for (int i = 0; i < DriveConfig.driveWheels.length; i++) {
                powers[i] /= maxPower;
            }
        }

        BaseOpMode.addData("Max Power", maxPower);
        // Command motor powers
        for (int i = 0; i < DriveConfig.driveWheels.length; i++) {
            DriveConfig.driveWheels[i].setPower(powers[i]);
            BaseOpMode.addData(String.format("Setting Motor %d to Power", i), powers[i]);
        }
    }

    public void move(double drive, double strafe, double turn, double speed) {
        move(new Vector(drive, strafe, turn).multiplied(speed));
    }
}

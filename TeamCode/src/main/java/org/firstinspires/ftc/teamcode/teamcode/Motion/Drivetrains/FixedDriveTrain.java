package org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.GeneralMatrix;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Matrix;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;

import org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.DriveConfig;


// A general fixed-wheel Holonomic drivetrain class
public class FixedDriveTrain extends Movement {

    public static Matrix toPowers = new GeneralMatrix(4, 3, new double[] {
            DriveConfig.DriveWheels.FR.x, DriveConfig.DriveWheels.FR.y, DriveConfig.DriveWheels.FR.h,
            DriveConfig.DriveWheels.FL.x, DriveConfig.DriveWheels.FL.y, DriveConfig.DriveWheels.FL.h,
            DriveConfig.DriveWheels.BR.x, DriveConfig.DriveWheels.BR.y, DriveConfig.DriveWheels.BR.h,
            DriveConfig.DriveWheels.BL.x, DriveConfig.DriveWheels.BL.y, DriveConfig.DriveWheels.BL.h,
    });


    public static Matrix h(double angle) {
        return new GeneralMatrix(3, 3,new double[]{
            Math.cos(angle), -Math.sin(angle), 0,
            Math.sin(angle), Math.cos(angle), 0,
            0, 0, 1
        });
    }


    public FixedDriveTrain(Vector startState) {
        super(startState);
    }

    public void move(Vector target) {
        move(target, false);
    }

    // Sets drivetrain power to a target power vector
    public void move(Vector target, boolean useFullPower) {

        double angle = loc.getPosition().get(2);
        Matrix h = h(-angle);
        Matrix Ph = toPowers.multiplied(h);
        Vector wheelVelocities = Ph.multiplied(loc.getDataVector());
        double xCorrection = DriveConfig.DriveWheels.Lxk*Math.signum(h.multiplied(loc.getDataVector()).get(0));
        Vector frictionCorrection = toPowers.multiplied(new Vector(xCorrection, 0, 0));

        for (int i = 0; i < 4; i++) {
            frictionCorrection.put(i, frictionCorrection.get(i) + Math.signum(wheelVelocities.get(i)) * DriveConfig.DriveWheels.Lmk);
        }

        Vector powers = Ph.multiplied(target);
        powers.add(frictionCorrection);

        /*double maxPower = 0;


        // Get power of each wheel with dot product
        for (int i = 0; i < driveWheels.length; i++) {
            if (Math.abs(powers.get(i)) > maxPower) maxPower = Math.abs(powers.get(i));
        }

        // Make sure that the speed is capped so that it doesn't go in the wrong direction
        if (maxPower > 1 || useFullPower) {
            for (int i = 0; i < driveWheels.length; i++) {
                powers[i] /= maxPower;
            }
        }

        BaseOpMode.addData("Max Power", maxPower);*/



        // Command motor powers
        for (int i = 0; i < driveWheels.length; i++) {
            driveWheels[i].setPower(powers.get(i));
            BaseOpMode.addData(String.format("Setting Motor %d to Power", i), powers.get(i));
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

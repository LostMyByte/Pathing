// Primary Author: Kieran Mattingly
package org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Localization.Location;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.DriveWheels;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Hardware;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.HardwareDevices.Motor;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.GeneralMatrix;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Matrix;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;


/**
 * A general, fixed-wheel drivetrain class
 */
public class FixedDriveTrain extends Movement {

    protected Motor[] driveWheels;

    // Converts from target powers to wheel powers
    public static Matrix toPowers = new GeneralMatrix(4, 3, new double[] {
            DriveWheels.FR.x, DriveWheels.FR.y, DriveWheels.FR.h,
            DriveWheels.FL.x, DriveWheels.FL.y, DriveWheels.FL.h,
            DriveWheels.BR.x, DriveWheels.BR.y, DriveWheels.BR.h,
            DriveWheels.BL.x, DriveWheels.BL.y, DriveWheels.BL.h,
    });


    /**
     * Position-only heading transform
     * @param angle
     * @return
     */
    public static Matrix h(double angle) {
        return new GeneralMatrix(3, 3,new double[]{
            Math.cos(angle), -Math.sin(angle), 0,
            Math.sin(angle), Math.cos(angle), 0,
            0, 0, 1
        });
    }


    Location loc;
    public FixedDriveTrain(Location loc) {
        this.loc = loc;

        driveWheels = new Motor[4];
        driveWheels[0] = new Motor(Hardware.rightFront);
        driveWheels[1] = new Motor(Hardware.leftFront);
        driveWheels[2] = new Motor(Hardware.rightBack);
        driveWheels[3] = new Motor(Hardware.leftBack);
    }

    @Override
    public void move(Vector target) {
        move(target, false, true);
    }

    /**
     * Sets drivetrain power to a target power (x, y, h) vector
     * @param target        Target power
     * @param useFullPower  Whether or not to use the max power
     * @param scalePowers   Scale powers down if they exceed 1
     */
    public void move(Vector target, boolean useFullPower, boolean scalePowers) {

        double angle = loc.getPosition().get(2);
        Matrix h = h(-angle);
        Matrix Ph = toPowers.multiplied(h);
        Vector wheelVelocities = Ph.multiplied(loc.getDataVector());
        double xCorrection = DriveWheels.Lxk*Math.signum(h.multiplied(loc.getDataVector()).get(0));
        Vector frictionCorrection = toPowers.multiplied(new Vector(xCorrection, 0, 0));

        for (int i = 0; i < 4; i++) {
            frictionCorrection.put(i, frictionCorrection.get(i) + Math.signum(wheelVelocities.get(i)) * DriveWheels.Lmk);
        }

        Vector powers = Ph.multiplied(target);
        powers.add(frictionCorrection);

        double maxPower = 0;


        // Get power of each wheel with dot product
        for (int i = 0; i < driveWheels.length; i++) {
            if (Math.abs(powers.get(i)) > maxPower) maxPower = Math.abs(powers.get(i));
        }

        // Make sure that the speed is capped so that it doesn't go in the wrong direction
        if ((maxPower > 1 || useFullPower) && !scalePowers) {
            for (int i = 0; i < driveWheels.length; i++) {
                powers.put(i, powers.get(i)/ maxPower);
            }
        }

        BaseOpMode.addData("Max Power", maxPower);



        // Command motor powers
        for (int i = 0; i < driveWheels.length; i++) {
            driveWheels[i].setPower(powers.get(i));
            BaseOpMode.addData(String.format("Setting Motor %d to Power", i), powers.get(i));
        }
    }

    /**
     * Move given a drive, strafe, turn, and speed command.
     * @param drive
     * @param strafe
     * @param turn
     * @param speed
     */
    public void move(double drive, double strafe, double turn, double speed) {
        move(new Vector(drive, strafe, turn).multiplied(speed));
    }

    /**
     * Directly set wheel powers
     * @param powers
     */
    public void moveRaw(Vector powers) {

        for (int i =0; i < driveWheels.length; i++) {
            driveWheels[i].setPower(powers.get(i));
        }
    }
}

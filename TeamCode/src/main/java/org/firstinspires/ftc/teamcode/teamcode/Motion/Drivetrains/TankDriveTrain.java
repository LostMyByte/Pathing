// Primary Author: Kieran Mattingly
package org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Hardware;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.HardwareDevices.Motor;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;

public class TankDriveTrain extends Movement {

    Motor L1;
    Motor L2;
    Motor R1;
    Motor R2;

    public TankDriveTrain(Vector startState) {
        super(startState);

        L1 = new Motor(Hardware.leftFront, true);
        L2 = new Motor(Hardware.leftBack, true);
        R1 = new Motor(Hardware.rightFront, false);
        R2 = new Motor(Hardware.rightBack, false);
    }

    public void setPowers(Vector powers) {

        BaseOpMode.addData("Power L", powers.get(0));
        BaseOpMode.addData("Power R", powers.get(1));
        this.L1.setPower(powers.get(0));
        this.L2.setPower(powers.get(0));
        this.R1.setPower(powers.get(1));
        this.R2.setPower(powers.get(1));
    }
    public void veryVeryDirectDrive(double drive, double turn, double speed){
        double powerLeft = (drive - turn)*speed;
        double powerRight = (drive + turn)*speed;

        L1.setPower(powerLeft);
        L2.setPower(powerLeft);
        R1.setPower(powerRight);
        R2.setPower(powerRight);
    }
    public void veryVeryDirectDrive(double drive, double turn){
        double powerLeft = (drive - turn);
        double powerRight = (drive + turn);

        L1.setPower(powerLeft);
        L2.setPower(powerLeft);
        R1.setPower(powerRight);
        R2.setPower(powerRight);
    }

    @Override
    public void move(Vector target) {
        Vector wheelPowers = new Vector(0, 0);

        wheelPowers.put(0, target.get(0) + target.get(1));
        wheelPowers.put(1, target.get(0) - target.get(1));

        moveRaw(wheelPowers);

    }

    public void moveRaw(Vector target) {
        setPowers(target);
    }




}

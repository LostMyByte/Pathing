// Primary Author: Kieran Mattingly
package org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Localization.Location;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Hardware;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.HardwareDevices.Motor;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;

public class TankDriveTrain extends Movement {

    Motor L1;
    Motor L2;
    Motor R1;
    Motor R2;

    public Location loc;

    public TankDriveTrain() {
        this.initialize();
    }
    private void initialize(){
        L1 = new Motor(Hardware.leftFront, true);
        L2 = new Motor(Hardware.leftBack, true);
        R1 = new Motor(Hardware.rightFront, false);
        R2 = new Motor(Hardware.rightBack, false);

        loc = new Location(0,0,0);
    }


    public void setPowers(Vector powers) {


        this.L1.setPower(powers.get(0));
        this.L2.setPower(powers.get(0));
        this.R1.setPower(powers.get(1));
        this.R2.setPower(powers.get(1));
    }

    @Override
    public void move(Vector target) {
        Vector wheelPowers = new Vector(0, 0);

        wheelPowers.put(0, target.get(0) + target.get(1));
        wheelPowers.put(1, target.get(0) - target.get(1));

        moveRaw(wheelPowers);
    }


    public void drive(double drive, double turn, double speed) {
        move(new Vector(-drive, -turn).multiplied(speed));
    }

    public void drive(double drive, double turn) {
        move(new Vector(-drive, turn));
    }

    public void moveRaw(Vector target) {
        //Location.input = target;
        setPowers(target);
    }




}

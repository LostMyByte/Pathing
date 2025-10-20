package org.firstinspires.ftc.teamcode.Motion.Drivetrains;

import org.firstinspires.ftc.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.Motion.Movement;
import org.firstinspires.ftc.teamcode.Motion.SystemModels.TankDrive;
import org.firstinspires.ftc.teamcode.Utilities.Configuration.DriveConfig;
import org.firstinspires.ftc.teamcode.Utilities.Configuration.Hardware;
import org.firstinspires.ftc.teamcode.Utilities.HardwareDevices.Motor;
import org.firstinspires.ftc.teamcode.Utilities.Math.Vector;

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

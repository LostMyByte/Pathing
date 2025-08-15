package org.firstinspires.ftc.teamcode.Motion.Drivetrains;


import org.firstinspires.ftc.teamcode.Utilities.HardwareDevives.Motor;
import org.firstinspires.ftc.teamcode.Utilities.Math.Vector;

public class DriveWheel extends Motor {

    // 3-Vector that represents the direction the wheel pushes in x, y, and h.
    public Vector MovementVector;

    public DriveWheel(String name, Vector movementVector) {
        super(name, false);
        this.MovementVector = movementVector;
    }
}

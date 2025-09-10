package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.TeliOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.KCP.DriveClasses.TankDrivetrain;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.MainIntake;

@TeleOp(name="Just Drive")
public class JustDrive extends BaseOpMode {

    TankDrivetrain drive;



    @Override
    public void externalInit() {
        drive = new TankDrivetrain();

        //drivetrain = new Drivetrain();
        //scoring = new Scoring();
        //intake = new MainIntake(hardwareMap);
    }

    @Override
    public void externalLoop() {
        double d = driver1.leftStick.Y();

        double turn = driver1.rightStick.X();
        double speed = 1;

        drive.move(d, turn, speed);

    }
}

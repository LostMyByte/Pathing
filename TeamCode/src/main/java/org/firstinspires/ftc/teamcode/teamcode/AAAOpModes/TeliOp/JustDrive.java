package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.TeliOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.MainIntake;

@TeleOp(name="Just Drive")
public class JustDrive extends BaseOpMode {

    //Drivetrain drivetrain;
    //Scoring scoring;
    MainIntake intake;


    @Override
    public void externalInit() {

        //drivetrain = new Drivetrain();
        //scoring = new Scoring();
        //intake = new MainIntake(hardwareMap);
    }

    @Override
    public void externalLoop() {
        double drive = driver1.leftStick.Y();
        double strafe = -driver1.leftStick.X();
        double turn = driver1.rightStick.X();
        double speed = 1;

        //drivetrain.drive(drive,strafe,turn,speed);
        BaseOpMode.addData("slidesLength", intake.getSlidesLength());
        if (driver1.dpad_up.isTapped()) {
            intake.increaseSlidesLengthInches(1);
        } else if (driver1.dpad_down.isTapped()) {
            intake.increaseSlidesLengthInches(-1);
        }
        if (driver1.cross.isPressed()) {
            intake.runIntake();
        } else if (driver1.circle.isPressed()){
            intake.reverseIntake();
        } else {
            intake.stopIntake();
        }
        BaseOpMode.addData("slidesLengthInches",intake.getSlidesLengthInches());
        BaseOpMode.addData("IntakeColor",intake.getIntakeColor());

    }
}

package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes;

import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.IntakeMagazine.IntakeMagazineStates.INTAKEACTIVE;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.IntakeMagazine.IntakeMagazineStates.INTAKEREVERSED;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.IntakeMagazine.IntakeMagazineStates.SHOOTING0;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Shooter.ShooterStates.ACTIVE;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Shooter.ShooterStates.NOTACTIVE;

import org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains.TankDriveTrain;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.IntakeMagazine;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Constants;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;

public class DimitriTeleop extends BaseOpMode{
    Shooter shooter;
    IntakeMagazine intake;
    TankDriveTrain drive;
    boolean shooterActive = true;

    @Override
    public void externalInit() {
        shooter = new Shooter(hardwareMap, 0, Constants.team);
        intake = new IntakeMagazine(hardwareMap);
        drive = new TankDriveTrain(new Vector(0,0));
    }

    @Override
    public void externalLoop() {
        drive.loc.updateOffsets();
        Vector target = new Vector(-driver1.leftStick.Y(), driver1.rightStick.X());
        drive.move(target);

        shooter.recieveOdoInputs(drive.loc.getPosX(), drive.loc.getPosY(), drive.loc.getPosH(), drive.loc.getTranslationalVelocity());

        if (shooterActive){
            shooter.setState(ACTIVE);
        } else {
            shooter.setState(NOTACTIVE);
        }

        if (driver1.dpad_up.isTapped()){
            shooterActive = !shooterActive;
        }

        if (driver1.square.isTapped()){
            intake.shootPurple();
        } else if (driver1.triangle.isTapped()){
            intake.shootGreen();
        } else if (driver1.cross.isTapped()){
            intake.setState(SHOOTING0);
        }

        if (driver1.rightTrigger.isPressed()) {
            intake.setState(INTAKEACTIVE);
        } else if (driver1.leftTrigger.isTapped()){
            intake.setState(INTAKEREVERSED);
        }

    }
}

package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.TeliOp;

import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.IntakeMagazine.IntakeMagazineStates.INTAKEACTIVE;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.IntakeMagazine.IntakeMagazineStates.INTAKEREVERSED;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.IntakeMagazine.IntakeMagazineStates.LOADED1;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.IntakeMagazine.IntakeMagazineStates.SHOOTING0;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.IntakeMagazine.IntakeMagazineStates.SHOOTING1;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.IntakeMagazine.IntakeMagazineStates.SHOOTING2;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Shooter.ShooterStates.ACTIVE;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Shooter.ShooterStates.NOTACTIVE;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Drivetrains.TankDriveTrain;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Localization.Location;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.IntakeMagazine;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.TeliOpDrivetrain;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Constants;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Math.Vector;

public class DimitriTeleop extends BaseOpMode{
    Shooter shooter;
    IntakeMagazine intake;
    TeliOpDrivetrain drive;
    boolean shooterActive = true;

    Location loc;

    @Override
    public void externalInit() {
        shooter = new Shooter(hardwareMap, 0, Constants.team);
        intake = new IntakeMagazine(hardwareMap);
        drive = new TeliOpDrivetrain(hardwareMap,0);
        loc = new Location(0, 0,0);
    }

    @Override
    public void externalLoop() {

        drive.drive(driver1.leftStick.Y(), driver1.rightStick.X());
        loc.update();
        shooter.recieveOdoInputs(loc.getPosX(), loc.getPosY(), loc.getPosH(), loc.getTranslationalVelocity());

        if (shooterActive){
            shooter.setState(ACTIVE);
        } else {
            shooter.setState(NOTACTIVE);
        }

        if (driver1.dpad_up.isTapped()){
            shooterActive = !shooterActive;
        }

        if (driver1.square.isTapped()){
            if (!intake.shootPurple()){
                driver1.rumble();
            }
        } else if (driver1.triangle.isTapped()){
            if (!intake.shootGreen()){
                driver1.rumble();
            }
        } else if (driver1.cross.isTapped()){
            intake.setState(SHOOTING0);
        }

        if (driver1.rightTrigger.isPressed()) {
            intake.setState(INTAKEACTIVE);
        } else if (driver1.leftTrigger.isPressed()){
            intake.setState(INTAKEREVERSED);
        }

        //if the intake is doing something and we are not commanding it to do something
        if ((intake.getState() == INTAKEACTIVE || intake.getState() == INTAKEREVERSED) && !(driver1.rightTrigger.isPressed() || driver1.leftTrigger.isPressed())){
            intake.setState(LOADED1);
            intake.setSensorReadState(IntakeMagazine.SensorReadStates.READING);
        }

    }
}

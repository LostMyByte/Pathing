// Primary Author: Dylan Cook, Mixed
/*
package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.TeliOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.MainIntake;
import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
@TeleOp(name="Dimitri Teleop 90")
@Disabled
public class DimitriTeleop90 extends BaseOpMode{

    Drivetrain drivetrain;

    MainIntake intake;
    double turretAngle = 0;
    double slidesLength = 0;
    double slidesSensitivityConstant = .65;
    ElapsedTime sweeperTimer;


    @Override
    public void externalInit() {

        drivetrain = new Drivetrain(hardware,0);



        intake = new MainIntake(hardware);
        intake.setState(MainIntake.IntakeStates.NOTHING);

        sweeperTimer = new ElapsedTime();
    }

    public void externalInitLoop(){

    }
    public enum Team{
        RED, BLUE, NOTDECLARED
    }
    @Override
    public void externalStart(){

        intake.setState(MainIntake.IntakeStates.HOMEUP);
    }



    @Override
    public void externalLoop(){
        double drive  = driver1.leftStick.Y();
        double strafe = -driver1.leftStick.X();
        double turn;
        double speed  = 1;
        boolean lockHeading;

        if(driver1.rightTrigger.isPressed()){
            speed = 0.5;
        }
        if (!driver1.leftBumper.isToggled()){
            turn = driver1.rightStick.X();
            lockHeading = false;
        }
        else {
            turn = 0;
            lockHeading = true;
        }



        drivetrain.drive(drive,strafe,turn,speed, lockHeading);

        if (intake.getState() == MainIntake.IntakeStates.HOMEUP && driver2.triangle.isTapped()){
            intake.setState(MainIntake.IntakeStates.EXTENDEDUP);
            slidesLength = 1.5;
            turretAngle = 0;
        } else if (intake.getState() == MainIntake.IntakeStates.HOMEDOWNNOTACTIVE && driver2.triangle.isTapped()){
            intake.setState(MainIntake.IntakeStates.EXTENDEDUP);
            slidesLength = 1.5;
            turretAngle = 0;
        } else if (driver2.square.isTapped()){
            intake.setState(MainIntake.IntakeStates.HOMEUP);
            turretAngle = 0;
        }


*/
/*
        else if (intake.getState() == MainIntake.IntakeStates.EXTENDEDUP && driver2.rightTrigger.isPressed()){
            intake.setState(MainIntake.IntakeStates.EXTENDEDACTIVE);
        } else if (intake.getState() == MainIntake.IntakeStates.HOMEDOWNNOTACTIVE && driver2.rightTrigger.isPressed()){
            intake.setState(MainIntake.IntakeStates.HOMEACTIVE);
        } else if (intake.getState() == MainIntake.IntakeStates.EXTENDEDUP && driver2.leftTrigger.isPressed()){
            intake.setState(MainIntake.IntakeStates.EXTENDEDREVERSED);
        } else if (intake.getState() == MainIntake.IntakeStates.HOMEDOWNNOTACTIVE && driver2.leftTrigger.isPressed()){
            intake.setState(MainIntake.IntakeStates.HOMEREVERSED);
        } else if (intake.getState() == MainIntake.IntakeStates.EXTENDEDACTIVE && !driver2.rightTrigger.isPressed()) {
            intake.setState(MainIntake.IntakeStates.EXTENDEDUP);
        } else if (intake.getState() == MainIntake.IntakeStates.EXTENDEDREVERSED && !driver2.leftTrigger.isPressed()) {
            intake.setState(MainIntake.IntakeStates.EXTENDEDUP);
        } else if (intake.getState() == MainIntake.IntakeStates.HOMEACTIVE && !driver2.rightTrigger.isPressed()) {
            intake.setState(MainIntake.IntakeStates.HOMEDOWNNOTACTIVE);
        } else if (intake.getState() == MainIntake.IntakeStates.HOMEREVERSED && !driver2.leftTrigger.isPressed()) {
                intake.setState(MainIntake.IntakeStates.HOMEDOWNNOTACTIVE);
        }


 *//*



        if (intake.getState() == MainIntake.IntakeStates.EXTENDEDACTIVE || intake.getState() == MainIntake.IntakeStates.EXTENDEDREVERSED  || intake.getState() == MainIntake.IntakeStates.EXTENDEDUP || intake.getState() == MainIntake.IntakeStates.EXTENDEDNOTACTIVE){
            slidesLength += driver2.leftStick.Y() * slidesSensitivityConstant;
            slidesLength = Range.clip(slidesLength, 0,17);
            intake.setSlidesLengthInches(slidesLength);
        }
        if (intake.getState() == MainIntake.IntakeStates.EXTENDEDACTIVE && intake.checkForSample()){
            driver2.rumble();
        } else {
            driver2.stopRumble();
        }

        if ((driver2.circle.isTapped() || sweeperTimer.seconds() < .4) && ((intake.getState() != MainIntake.IntakeStates.EXTENDEDACTIVE || intake.getState() != MainIntake.IntakeStates.EXTENDEDNOTACTIVE || intake.getState() != MainIntake.IntakeStates.EXTENDEDREVERSED) || (intake.getSlidesLengthInches() > 5))){
            intake.sweeperOut();
        } else {
            intake.sweeperIn();
        }

        if (driver2.circle.isTapped()){
            sweeperTimer.reset();
        }

        BaseOpMode.addData("Intake State", intake.getState());
        //BaseOpMode.addData("Scoring State", scoring.getState());
        BaseOpMode.addData("Slides Length", intake.getSlidesLengthInches());
        BaseOpMode.addData("Intake State", intake.getState());


    }
}
*/

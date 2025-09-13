package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.Autonomous.TestAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.KCP.Localization.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Hardware;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.MainIntake;

@Autonomous(name = "Reset Stufsf")
public class ResetOffsetsAuto extends BaseOpMode {
    @Override
    public void externalInit() {
        Constants.startAngle = Double.NaN;
        Scoring.encoderPosition = 0;
        GoBildaPinpointDriver gyro = hardware.get(GoBildaPinpointDriver.class, Hardware.odoWheels);
        gyro.resetPosAndIMU();
        Scoring scoring = new Scoring(hardware);
        scoring.setState(Scoring.ScoringStates.HOME);

        MainIntake intake = new MainIntake(hardware);
        intake.setState(MainIntake.IntakeStates.HOMEUP);
    }

    @Override
    public void externalLoop() {

    }
}

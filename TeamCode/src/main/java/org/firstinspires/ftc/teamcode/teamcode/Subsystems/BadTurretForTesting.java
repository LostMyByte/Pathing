package org.firstinspires.ftc.teamcode.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Controllers.PID;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Localization.LimeLightData;
import org.firstinspires.ftc.teamcode.teamcode.Motion.Signals.SetItSignal;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration.Hardware;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.HardwareDevices.Motor;

@Config
public class BadTurretForTesting extends Subsystem {

    public static PID.PIDCoefficients coefficients = new PID.PIDCoefficients(-3., 0, -0., 0, 0);
    SetItSignal encoder;
    SetItSignal target;
    Motor motor;
    PID motorController;

    double angle = 0;

    public void setAngle(double theta) { angle = theta;
        BaseOpMode.addData("Turret Target", angle);
    }

    public BadTurretForTesting() {
        this.motor = new Motor(Hardware.turret, true, true, -0.01);
        this.encoder = new SetItSignal(1);
        this.target = new SetItSignal(1);
        this.motorController = new PID(encoder, target, coefficients);
    }

    @Override
    public void update() {
        motor.setPower(motorController.getCorrection().get(0));
    }

    @Override
    public void updateSensors() {
        LimeLightData.turretAngle = motor.encoder.getPosition();
        encoder.set(motor.encoder.getPosition());
        target.set(angle);
    }
}

package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.TeliOp;

import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;

import org.firstinspires.ftc.teamcode.teamcode.KCP.Localization.AprilTagOdometrySource;
import org.firstinspires.ftc.teamcode.teamcode.KCP.Localization.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.teamcode.KCP.Movement;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.MainIntake;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Servos;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Dash.DashPositions;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.HardwareDevices.Motor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name="Test Teleop", group="Iterative Opmode")
public class TestOpMode extends BaseOpMode {


    Shooter shooter;

    @Override
    public void externalInit() {
        shooter = new Shooter(hardwareMap, 0, Constants.Team.BLUE);
    }

    @Override
    public void externalLoop() {
        shooter.setState(Shooter.ShooterStates.SHOOTERTESTING);

    }
}
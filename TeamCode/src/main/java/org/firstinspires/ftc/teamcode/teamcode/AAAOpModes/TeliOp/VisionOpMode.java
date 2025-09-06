package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.TeliOp;

import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.MainIntake.IntakeStates.SEARCHJUSTYELLOW;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teamcode.KCP.Movement;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Constants;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.MainIntake;

import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Drivetrain;
import org.opencv.videoio.VideoCapture;


@TeleOp(name = "KieranTeleOP")
public class VisionOpMode extends BaseOpMode {
    private Drivetrain drivetrain;
    private Movement movement;
    private FtcDashboard dash;
    public boolean targeting = false;

    VideoCapture webcam;
    //VisionPortal vision;
    //IntoTheDeepVisionProcessor processor;

    MainIntake intake;

    double[] holdingPosition = null;



    @Config
    public static class holdPositionFilter {
        public static double alpha = 0.1;
    }




    @Override
    public void externalInit() {

        Constants.team = Constants.Team.BLUE;
        //webcam = hardware.get(WebcamName.class, "Webcam 1");
        //processor = new IntoTheDeepVisionProcessor();
        //vision = new VisionPortal.Builder().setCameraResolution(new Size(1280, 720)).addProcessor(processor).setCamera(webcam).build();
        intake = new MainIntake(hardware);




        dash = FtcDashboard.getInstance();
        telemetry = dash.getTelemetry();


        //drivetrain = new Drivetrain(0);
        movement = new Movement(0,0,0);

        intake.setState(MainIntake.IntakeStates.HOMEUP);
        intake.activeMode = MainIntake.SearchMode.SlidesYDriveX;
        movement.setActiveCorrectionMethod(Movement.CorrectionMethods.Profiled);

    }

    long numAverages = 0;

    double holdY = 0;
    double holdX = 0;
    double oldOffset = 0;


    @Override
    public void externalLoop() {
        //drivetrain.update();
        //drivetrain.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, 1-gamepad1.right_trigger);

        if (gamepad1.square) intake.setState(SEARCHJUSTYELLOW);


        //telemetry.addData("Blue Center", limelight.centerBlue);
        //telemetry.addData("Blue Angle", limelight.angleBlue);

        if (intake.getState() == SEARCHJUSTYELLOW) {
            if (intake.blockMovement != oldOffset) {
                oldOffset = intake.blockMovement;
                holdX += MainIntake.CameraMount.DriveAlpha*(movement.getX() + intake.blockMovement - holdX);
            }
            BaseOpMode.addData("Holding X", holdX);
            BaseOpMode.addData("Holding Y", holdY);
            movement.holdPosition(holdX, holdY, 0, 1);
        }
        else movement.holdPosition(0, 0, 0, 1);
        BaseOpMode.addData("X", movement.getX());
        BaseOpMode.addData("Y", movement.getY());
        BaseOpMode.addData("Intake Sample Color:", intake.getIntakeColor());
        telemetry.update();


    }
}

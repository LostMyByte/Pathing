package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.TeliOp;


import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Constants.Team.BLUE;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Constants.Team.RED;
import static org.firstinspires.ftc.teamcode.teamcode.Subsystems.Constants.team;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;



import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Constants;


import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.Control.PID;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.HardwareDevices.Motor;
import org.firstinspires.ftc.teamcode.teamcode.Utilities.HardwareDevices.MotorEncoder;

import java.util.List;


@TeleOp(name = "VisionTest")
public class VisionOpMode extends BaseOpMode {

    PID turretPDL;
    double heading = 0;
    double turretTargetAngle;

    private FtcDashboard dash;

    Limelight3A limelight;

  //  Drivetrain drivetrain;

    CRServo turret;

    Motor turretEncoder;

    double visionDeadzone = 10;


    @Override
    public void externalInit() {

        team = BLUE;
      //  drivetrain = new Drivetrain(hardwareMap,0);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // make number higher to get more data
        limelight.pipelineSwitch(0);
        limelight.start();

        limelight.reloadPipeline();
        turretEncoder = hardwareMap.get(Motor.class, "encoder");
        turret = hardwareMap.get(CRServo.class, "left");

// we can use this so the limelight gives better data
        // double robotYaw = imu.getAngularOrientation().firstAngle;
        // limelight.updateRobotOrientation(robotYaw);



        dash = FtcDashboard.getInstance();
        telemetry = dash.getTelemetry();


    }




    @Override
    public void externalLoop() {
        /*LLResult result = limelight.getLatestResult();
        int id = 0;
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            id = fiducial.getFiducialId(); // The ID number of the fiducial
            double degreesXtoApriltag = fiducial.getTargetXDegrees();
            //Pose3D distance = fiducial.getCameraPoseTargetSpace();
           // double xDistance = (fiducial.getRobotPoseTargetSpace().getPosition().x)*100;
         //   double yDistance = (fiducial.getRobotPoseTargetSpace().getPosition().y)*100;
            double ty = limelight.getLatestResult().getTy();
            double tx = limelight.getLatestResult().getTx();


               *//* if(id==21){
                    Constants.motif = Constants.Motif.GPP;
                } else if (id==22){
                    Constants.motif = Constants.Motif.PGP;
                } else if (id==23){
                    Constants.motif = Constants.Motif.PPG;
                }
*//*
            multTelemetry.addData("id",id);
            multTelemetry.addData("degrees", degreesXtoApriltag);
            multTelemetry.addData("dist across", tx);
            multTelemetry.addData("dist away", ty);
            if(limelight.getLatestResult().isValid()){
                if(team == BLUE||id == 20){
                    if(Math.abs(degreesXtoApriltag) > visionDeadzone){
                         turret.setPower(-degreesXtoApriltag/10);
            }}else if (team == RED || id ==24){
                    if(Math.abs(degreesXtoApriltag) > visionDeadzone){
                        turret.setPower(-degreesXtoApriltag/10);
                }}}

        }
      //  drivetrain.nonDriverOrientedDrive(driver1.leftStick.Y(), -driver1.leftStick.X(), driver1.rightStick.X());

        multTelemetry.addData("isRunning", limelight.isRunning());
        multTelemetry.addData("power", turret.getPower());
        multTelemetry.addData("id",id);
*/

        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int id = fiducial.getFiducialId(); // The ID number of the fiducial
            double degreesXtoApriltag = fiducial.getTargetXDegrees(); //gets angle to limelight along x plane

            double ty = limelight.getLatestResult().getTy(); // gets degrees to crosshair from primary target along y axis
            double tx = limelight.getLatestResult().getTx();// gets degrees to crosshair from primary target along x axis

            if( team == BLUE || id == 20){
                heading = tx;
            } else if (team == RED || id == 24){
                heading = tx;
            }
        }


        turretTargetAngle = /*Math.asin(xVelocity/getBallSpeed())*/ - heading;//sets target angle to face goal. does this by setting target angle the negitive heading (current difference in degrees from target) and also accounts for fact that robot moves

        while (turretTargetAngle > Math.PI){
            turretTargetAngle -= 2*Math.PI;
        }
        while (turretTargetAngle < -Math.PI){
            turretTargetAngle += 2*Math.PI;
        }
        double currentAngle = turretEncoder.encoder.getPosition();
        //convert this to radians and wrap the angle
        double ticksPerRotation=8130;
        currentAngle = currentAngle * (2*Math.PI/ticksPerRotation);

        while (currentAngle > Math.PI){
            currentAngle -= 2*Math.PI;
        }
        while (currentAngle < -Math.PI){
            currentAngle += 2*Math.PI;
        }

        turretPDL.setConstants(0,0,0);
        turretPDL.getCorrectionHeading(currentAngle,turretTargetAngle);

        multTelemetry.addData("isRunning", limelight.isRunning());
        multTelemetry.addData("power", turret.getPower());
        multTelemetry.addData("ticks", turretEncoder.encoder.getPosition());
        multTelemetry.addData("targetAngle", turretTargetAngle);

      //  turret.setPower((turretPDL.getCorrectionHeading(currentAngle,turretTargetAngle)/1));


    }
}

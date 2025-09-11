package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.TeliOp;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Constants;


import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.teamcode.Subsystems.Drivetrain;

import java.util.List;


@TeleOp(name = "VisionTest")
public class VisionOpMode extends BaseOpMode {

    private FtcDashboard dash;

    Limelight3A limelight;

    Drivetrain drivetrain;




    @Override
    public void externalInit() {

        Constants.team = Constants.Team.BLUE;
        drivetrain = new Drivetrain(hardwareMap,0);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // make number higher to get more data
        limelight.start();
        limelight.reloadPipeline();

// we can use this so the limelight gives better data
        // double robotYaw = imu.getAngularOrientation().firstAngle;
        // limelight.updateRobotOrientation(robotYaw);



        dash = FtcDashboard.getInstance();
        telemetry = dash.getTelemetry();


    }




    @Override
    public void externalLoop() {
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int id = fiducial.getFiducialId(); // The ID number of the fiducial
            double degreesXtoApriltag = fiducial.getTargetXDegrees();
            //Pose3D distance = fiducial.getCameraPoseTargetSpace();
            double xDistance = (fiducial.getRobotPoseTargetSpace().getPosition().x)*100;
            double yDistance = (fiducial.getRobotPoseTargetSpace().getPosition().y)*100;



               /* if(id==21){
                    Constants.motif = Constants.Motif.GPP;
                } else if (id==22){
                    Constants.motif = Constants.Motif.PGP;
                } else if (id==23){
                    Constants.motif = Constants.Motif.PPG;
                }
*/
            multTelemetry.addData("id",id);
            multTelemetry.addData("degrees", degreesXtoApriltag);
            multTelemetry.addData("dist across", xDistance);
            multTelemetry.addData("dist away", yDistance);

        }
        drivetrain.nonDriverOrientedDrive(driver1.leftStick.Y(), -driver1.leftStick.X(), driver1.rightStick.X());
        telemetry.addData("gjfdjgfv", limelight.isRunning());



    }
}

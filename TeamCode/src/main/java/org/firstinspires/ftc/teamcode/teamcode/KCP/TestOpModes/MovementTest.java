package org.firstinspires.ftc.teamcode.KCP.TestOpModes;//package org.firstinspires.ftc.teamcode.KCP.TestOpModes;
//
//
//
//import static org.firstinspires.ftc.teamcode.AAAOpModes.Autonomous.Paths.Path.*;
//
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
//import org.firstinspires.ftc.teamcode.teamcode.KCP.Localization.Location;
//import org.firstinspires.ftc.teamcode.teamcode.KCP.Movement;
//import org.firstinspires.ftc.teamcode.AAAOpModes.Autonomous.Paths;
//
//@Autonomous(name="Movement Test", group="Testing")
//public class MovementTest extends OpMode {
//
//
//    double power = 1 * .6;
//
//    boolean pressed1 = false;
//    boolean pressed2 = false;
//
//    boolean pathComplete = false;
//
//    Movement drive;
//    public void init() {
//
//        BaseOpMode.hardware = hardwareMap;
//
//        MTPath1.compile();
//        MTPath2.compile();
//        MTPath3.compile();
//        MTPath4.compile();
//        MTPath5.compile();
//        MTPath6.compile();
//
//        DriveToSpike1.compile();
//        DriveToSpike2.compile();
//        DriveToSpike3.compile();
//        DriveToStack1.compile();
//        DriveToStack2.compile();
//        DriveToStack3.compile();
//
//        drive = new Movement(60,15);
//
//        Backwards80.compile();
//        Forward80.compile();
//
//        initialize();
//    }
//
//    private void initialize(){
//        telemetry.addData("status","initialized");
//        telemetry.update();
//    }
//
//    public void loop() {
////        holdOrigin();
////        movementTest();
////        checkOdoDrift();
////        driveForwardAndBack();c
////        checkHoldPositionPID();
//
//        drive.drive.directDrive(0,.5,0,0);
//
//        telemetry.addData("X", Location.x());
//        telemetry.addData("Y", Location.y());
//        telemetry.addData("H", Location.heading());
//
//        drive.update();
//        telemetry.update();
//    }
//
//    private void holdOrigin(){
//
//        drive.holdPosition(0,0,0);
//        telemetry.addLine("Holding Position");
//
//    }
//
//    private void checkHoldPositionPID(){
//
//        if(gamepad1.dpad_up && !pressed1){
//            telemetry.addLine("Holding Position 1");
//            drive.shiftLocation(0,50);
//            pressed1 = true;
//        }else if(!gamepad1.dpad_up) {
//            telemetry.addLine("Holding Position 2");
//            pressed1 = false;
//            drive.shiftLocation(0,-50);
//        }
//
//    }
//
//    private void controlSwervePod(){
//
////            Vector2d stick = new Vector2d(-gamepad1.left_stick_x, -gamepad1.left_stick_y);
////            System.out.println("stickx : " + stick.movementTestX + "stick movementTestY" + stick.movementTestY);
////            drive.facePodDirection(stick, gamepad1.right_trigger);
////            drive.update();
//
//    }
//
//    private void checkOdoDrift(){
//            drive.update();
//            if(!pressed1 && gamepad1.cross){
//                if(pressed2){
//                    pressed2 = false;
//                }else{
//                    pressed2 = true;
//                }
//                pressed1 = true;
//                pathComplete = false;
//            }else if(!gamepad1.cross){
//                pressed1 = false;
//            }
//
//            telemetry.addData("X", Location.x());
//            telemetry.addData("Y", Location.y());
//            telemetry.update();
//
//            if(!pressed2){
//                telemetry.addLine("Holding Position Turned");
//
//                drive.holdPosition(0,0, 2*Math.PI/3);
//            }else{
//                telemetry.addLine("Holding Position Straight");
//
//                drive.holdPosition(0,0, 0);
//            }
//    }
//
//    private void driveForwardAndBack(){
//        drive.update();
//        if(!pressed1 && gamepad1.cross){
//            if(pressed2){
//                pressed2 = false;
//            }else{
//                pressed2 = true;
//            }
//            pressed1 = true;
//            pathComplete = false;
//            Paths.Path.Backwards80.resetPath();
//            Paths.Path.Forward80.resetPath();
//        }else if(!gamepad1.cross){
//            pressed1 = false;
//        }
//
//        telemetry.addData("X", Location.x());
//        telemetry.addData("Y", Location.y());
//        telemetry.update();
//
//        if(!pressed2 && !pathComplete){
//            telemetry.addLine("Driving Backward");
//
//            pathComplete = !drive.followPath(Paths.Path.Backwards80, .55, 0, 0, true);
//        }else if(pressed2 && !pathComplete){
//
//            telemetry.addLine("Driving Forward");
//
//            pathComplete = !drive.followPath(Paths.Path.Forward80, .55, 0, 0, true);
//        }else{
//            if(!pressed2){
//                telemetry.addLine("Holding Backward");
//
//                drive.holdPosition(0,-130, 0);
//            }else{
//                telemetry.addLine("Holding Forward");
//
//                drive.holdPosition(0,0, 0);
//            }
//        }
//    }
//
//    int pathNumber = 1;
//    private boolean paths(){
//        switch (pathNumber) {
//            case 1:
//                telemetry.addLine("Following Path 1");
//
//                return !drive.followPath(MTPath1, power/4 + .5, 0, .8);
//            case 2:
//                telemetry.addLine("Following Path 2");
//
//                return !drive.followPath(MTPath2, power/2 + .5, 0, .8);
//            case 3:
//                telemetry.addLine("Following Path 3");
//
//                return !drive.followPath(MTPath3, power, 0, .8);
//            case 4:
//                telemetry.addLine("Following Path 4");
//
//                return !drive.followPath(MTPath4, power, 0, .8);
//            case 5:
//                telemetry.addLine("Following Path 5");
//
//                return !drive.followPath(MTPath5, power, 0, .8);
//            case 6:
//                telemetry.addLine("Following Path 6");
//
//                return !drive.followPath(MTPath6, power, 0, 0, true);
//            default:
//                telemetry.addLine("Holding Start Position");
//
//                drive.holdPosition(0, 0, 0);
//                return false;
//        }
//    }
//
//    private void movementTest() {
//        if(paths()){
//            pathNumber++;
//        }
//        drive.update();
//    }
//
//    private boolean testPaths(){
//        switch (pathNumber) {
//            case 1:
//                telemetry.addLine("Following Path 1");
//
//                return !drive.followPath(DriveToSpike1, power, 0, 0, true);
//            case 7:
//                telemetry.addLine("Following Path 2");
//
//                return !drive.followPath(DriveToSpike2, power, 0, 0, true);
//            case 4:
//                telemetry.addLine("Following Path 3");
//
//                return !drive.followPath(DriveToSpike3, power, 0, 0, true);
//            case 3:
//                telemetry.addLine("Following Path 4");
//
//                return !drive.followPath(DriveToStack1, power, 0, 0, true);
//            case 5:
//                telemetry.addLine("Following Path 5");
//
//                return !drive.followPath(DriveToStack2, power, 0, 0, true);
//            case 6:
//                telemetry.addLine("Following Path 6");
//
//                return !drive.followPath(DriveToStack3, power, 0, 0, true);
//
//            case 2:
//                telemetry.addLine("Following Path 5");
//
//                return !drive.followPath(Spike1ToBackdrop, power, 0, 0, true);
//
//                default:
//                telemetry.addLine("Holding Start Position");
//
//                drive.stopDrive();
//                return false;
//        }
//    }
//
//    private void pathTest() {
//        if(testPaths()){
//            pathNumber++;
//        }
//        drive.update();
//    }
//
////    public void runOpMode() {
//
//
////
////
////
//
////
//
////
//
////        while (opModeIsActive()){
////
////            }
////  Y         if(game
////  telemetry.addData("Heading", location[2]);pad1.dpad_down && !pressed2){
////                drive.odo.setPosition(location[0], location[1]+50);
////                pressed2 = true;
////            }else if(!gamepad1.dpad_down){
////                pressed2 = false;
////            }
//////            drive.movementPID.setConstants(MovementV2.MovementDash.holdMoveP, MovementV2.MovementDash.holdMoveI, MovementV2.MovementDash.holdMoveD);
////            drive.holdMovementPID.update(-location[1]);
////            drive.headingPID.update(drive.catchJump(0, -drive.getLocation()[2]));
////            drive.drive(0, 1, location[2], drive.holdMovementPID.getCorrection(), drive.headingPID.getCorrection());
////  Y     }
//
//
////
////        telemetry.addData("Heading", location[2]);while (opModeIsActive()){
////            drive.odo.localize();
////            drive.moveYentPID.update((M
////            telemetry.addData("Heading", location[2]);ath.sqrt(Math.pow(drive.getLocation()[0], 2) + Math.pow(drive.getLocation()[1], 2))));
////            drive.headingPID.update(drive.catchJump(0, -drive.getLocation()[2]));
////            drive.drive(0,1,drive.getLocation()[2], drive.movementPID.getCorrection(), drive.headingPID.getCorrection());
////        }
//
//}

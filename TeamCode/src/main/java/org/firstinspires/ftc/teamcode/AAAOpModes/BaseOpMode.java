package org.firstinspires.ftc.teamcode.AAAOpModes;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Motion.Controllers.Signal;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Utilities.HardwareDevives.Motor;
/*import org.firstinspires.ftc.teamcode.Utilities.HardwareDevices.Controller;
import org.firstinspires.ftc.teamcode.Utilities.HardwareDevices.Gyro;
import org.firstinspires.ftc.teamcode.Utilities.HardwareDevices.Motor;
import org.firstinspires.ftc.teamcode.Utilities.HardwareDevices.MotorEncoder;
import org.firstinspires.ftc.teamcode.Utilities.HardwareDevices.Servo;
import org.firstinspires.ftc.teamcode.Utilities.Telemetry.RobotLogTelemetry;
import org.firstinspires.ftc.teamcode.Utilities.Telemetry.ThrowbackTelemetry;
import org.firstinspires.ftc.teamcode.Utilities.Telemetry.VoidTelemetry;
*/
import java.util.List;

public abstract class BaseOpMode extends LinearOpMode {

    public static HardwareMap hardware;
    public static Telemetry telemetree;
    public static FtcDashboard dashboard = FtcDashboard.getInstance();
    public static Telemetry dashboardTelemetry = dashboard.getTelemetry();
  // public static ThrowbackTelemetry multTelemetry;
    public static boolean isActive;

    //public static Controller driver1, driver2;
    public static List<LynxModule> allHubs;

    /**
     * Initialize opMode Utilities
     * @param opMode
     */
    public static void setOpMode(BaseOpMode opMode){

        isActive = false;

        hardware = opMode.hardwareMap;

        telemetree = opMode.telemetry;
        telemetree.setMsTransmissionInterval(5);
      //  multTelemetry = new ThrowbackTelemetry(new MultipleTelemetry(telemetree, dashboardTelemetry));

        //driver1 = new Controller(opMode.gamepad1);
        //driver2 = new Controller(opMode.gamepad2);

        allHubs = hardware.getAll(LynxModule.class);

        for(LynxModule hub : allHubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

    }

    /**
     * Initialize Servos and other code necessary to run the robot
     * Will execute once when the init button is pressed
     */
    public abstract void externalInit();

    /**
     * Will repeat while the play button is not pressed
     */
    public void externalInitLoop(){
    //    Gyro.zeroAngles();
    }

    /**
     * Will execute when the play button is pressed once
     */
    public void externalStart(){}

    /**
     * What will run while the play button is pressed
     */
    public abstract void externalLoop();

    /**
     * Will execute once when the stop button is pressed
     */
    public void externalStop(){
        //Motor.stopMotors();
    }

    @Override
    public void runOpMode(){



        initializeUtilities();
        externalInit();

        Signal.startALl();


        do{
            //update sensors
            updateUtilities();
            //run loop
            externalInitLoop();
            //command motors and servos

        }while (opModeInInit());


        externalStart();

        while (opModeIsActive()){
            updateUtilities();
            externalLoop();
            isActive = true;
        }
        externalStop();

    }

    private void initializeUtilities(){
        Signal.signals.clear();
        Subsystem.resetSubsystemList();
        Motor.resetMotorList();
        /*Servo.resetServoList();

        MotorEncoder.resetEncoderList();
        Gyro.resetGyroList();*/

        setOpMode(this);
    }

    private void updateUtilities(){
        Signal.updateAll();
        //update telemetry
        updateTelemetry();
        //lastly command powers
        Subsystem.updateSubsystems();
        Motor.commandPowers();
        /*Servo.commandPositions();*/
        //begin by updating sensors
        updateControllers();
        Subsystem.pingSensors();
        /*MotorEncoder.updateEncoders();
        Gyro.updateAngles();*/

    }

    /**
     * Telemetry print data
     * @param s - identifier
     * @param o - data
     */
    public static void addData(String s, Object o){
        //multTelemetry.addData(s, o);
        dashboardTelemetry.addData(s, o);
    }

    /**
     * Telemetry print string
     * @param o - string
     */
    public static void addLine(Object o){
        //multTelemetry.addLine((String) o);
        dashboardTelemetry.addLine((String) o);
    }

    /**
     * Console print
     * @param s - string
     */
    public static void printlnToComputer(String s){
        System.out.println(s);
    }

    public static boolean isInInit(){
        return isInInit();
    }
    public static boolean isActive(){
        return isActive;
    }

    public static boolean mainThreadRunning(){
        return isInInit() || isActive();
    }

    public static void setNoTelemetry(){
        //multTelemetry.setTelemetry(new VoidTelemetry());
    }

    public static void setRobotLogTelemetry(){
        //multTelemetry.setTelemetry(new RobotLogTelemetry());
    }
    public static void updateTelemetry(){
        //multTelemetry.update();
        dashboardTelemetry.update();
    }

    public static void updateControllers(){
        //driver1.update(); driver2.update();
    }

    public static HardwareMap getHardwareMap(){
        return hardware;
    }
}

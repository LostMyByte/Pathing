// Primary Author: Past Team Members, Mixed
package org.firstinspires.ftc.teamcode.teamcode.Utilities.HardwareDevices;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;

import java.util.ArrayList;

public class Gyro {

    private final BNO055IMU controlHubIMU;

    public static ArrayList<Gyro> gyros = new ArrayList<>();
    private double wrappedHeading = 0;
    private double rawHeading = 0;
    private double offset = 0;
    double previousTime = 0;
    double previousChange = 0;

//Problem:
    //When Gyro reads -3.14 or 3.14 and goes over, it
    // goes to negative version of that which causes it to spin in a circle to reach what it was at before

    public Gyro(String name){
        gyros.add(this);
//        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.DOWN;
//        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
//        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        controlHubIMU = BaseOpMode.hardware.get(BNO055IMU.class, name);
//        controlHubIMU.initialize(new IMU.Parameters(orientationOnRobot));

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json";
        controlHubIMU.initialize(parameters);


        update();

        setCurrentHeading(0);


    }


    public double getHeading(){
        return wrappedHeading;
    }


    //maybe make total heading
    public double getRawHeading(){
        return rawHeading;
    }

    public double getUnwrappedHeading(){
        return rawHeading - offset;
    }

    public void resetHeading(){
        BaseOpMode.addData("Raw", rawHeading);
        offset = rawHeading;
    }



    //In radians
    public void setCurrentHeading(double heading){
        resetHeading();
        offset -= heading;
    }

    public static void updateAngles(){
        for(Gyro g : gyros){
            g.update();
        }
    }

    public static void zeroAngles(){
        for(Gyro g : gyros){
            g.resetHeading();
        }
    }

    public double getVeryRawHeading(){
        return 0;
    }

    public double getRateOfChange(){
            double change = getHeading();
            double deltaTime = (System.currentTimeMillis() - previousTime) / 1000.0;
            double deltaChange = change - previousChange;
            double rateOfChange = deltaChange/deltaTime;
            previousTime = System.currentTimeMillis();
            previousChange = change;
            return Math.abs(rateOfChange);
    }

    public static void resetGyroList(){
        gyros.clear();
    }

    private void update(){

        //TODO revHub orientation might matter
        Orientation angles = controlHubIMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        rawHeading = angles.firstAngle;

        wrappedHeading = wrapAngle(rawHeading - offset);
//        BaseOpMode.addData("offset", offset);

    }

    private double wrapAngle(double angle){
//        angle += 2 * Math.PI;

        while (angle > 2 * Math.PI){
            angle -= 2 * Math.PI;
        }

        while (angle < 0){
            angle += 2 * Math.PI;
        }




        return angle;
    }




//    private BHI260IMU imuConstructor(String deviceName){
//        BHI260IMU imu;
//        imu = BaseOpMode.hardware.get(BHI260IMU.class, deviceName);
//        BHI260IMU.Parameters IMUparameters;
//
//        IMUparameters = new BHI260IMU.Parameters(
//                new RevHubOrientationOnRobot(
//                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
//                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
//                )
//        );
//        imu.initialize(IMUparameters);
//        return imu;
//    }
}

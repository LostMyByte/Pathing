package org.firstinspires.ftc.teamcode.teamcode.Utilities.yHardware;

//hardware variables followed by hardware objects

//DCMotorEx: https://ftctechnh.github.io/ftc_app/doc/javadoc/index.html?com/qualcomm/robotcore/hardware/DcMotorEx.html

import org.firstinspires.ftc.teamcode.teamcode.Utilities.zLibraries.Utilities.Vector2d;

public class V7Hardware extends Hardware{

    protected static final String
            verticalEncoder = "verticalEncoder", horizontalEncoder = "horizontalEncoder";

    private static final double verticalEncoderTicksToCM = -0.002414, horizontalEncoderTicksToCM = 0.0018948;

    private static final String
            liftMotor1 = "dr4b", liftMotor2 = verticalEncoder,
            intakeMotor1 = "intakeslides", intakeMotor2 = horizontalEncoder;

    private static final String
            leftFront = "drivefl", rightFront  = "drivefr",
            leftBack = "drivebl", rightBack = "drivebr";

    private static final Vector2d mecanumWheelPowerVector = new Vector2d(-.11,.16);

    public static String getLiftMotor2(){return liftMotor2;}

    public static String getLiftMotor1(){return liftMotor1;}

    public static String getIntakeMotor1(){return intakeMotor1;}

    public static String getIntakeMotor2(){return intakeMotor2;}

    public static String getRightFront(){return rightFront;}

    public static String getRightBack(){return rightBack;}

    public static String getLeftFront(){return leftFront;}

    public static String getLeftBack(){return leftBack;}

    public static Vector2d getMecanumWheelPowerVector(){
        return mecanumWheelPowerVector;
    }
    public static String getVerticalEncoder(){
        return verticalEncoder;
    }
    public static String getHorizontalEncoder(){
        return horizontalEncoder;
    }

    public static double getVerticalEncoderTicksToCM(){
        return verticalEncoderTicksToCM;
    }
    public static double getHorizontalEncoderTicksToCM(){
        return horizontalEncoderTicksToCM;
    }
}

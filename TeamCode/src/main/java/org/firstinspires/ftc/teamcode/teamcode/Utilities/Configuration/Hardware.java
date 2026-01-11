// Primary Author: Mixed
package org.firstinspires.ftc.teamcode.teamcode.Utilities.Configuration;

//hardware variables followed by hardware objects

//DCMotorEx: https://ftctechnh.github.io/ftc_app/doc/javadoc/index.html?com/qualcomm/robotcore/hardware/DcMotorEx.html

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Hardware {


    public static final Size cameraResolution = new Size(1280, 720);

    public static final String odoWheels = "odoWheels";

    public static final String
            leftFront = "fl", rightFront = "fr",
            leftBack = "bl", rightBack = "br";

    public static final String indexServo1 = "IndexServo1", indexServo2 = "shub0red", shooterDoor = "shooterDoor";

    public static final String hood = "shub5purple";

    public static final String mecanum1 = "mecanum1", mecanum2 = "mecanum2";
    public static final String frontIntake = "ehub2green", rearIntake = "chub2orange";
    public static final String shooter1 = "chub3purple", shooter2 = "ehub3black ";
    public static final String transfer = "transfer";
    public static final String turret = "shub1gray", turret2 = "shub4black";
    public static final String fliPrampFront = "shub3brown", fliPrampRear = "shub2green";

}

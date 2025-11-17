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

    public static final String magazineL = "magazineL", magazineR = "magazineR", shooterDoor = "shooterDoor";

    public static final String hood = "hood";

    public static final String mecanum1 = "mecanum1", mecanum2 = "mecanum2";
    public static final String intake = "intake";
    public static final String shooter1 = "shooter1", shooter2 = "shooter2";
    public static final String transfer = "transfer";
    public static final String turret = "turret";

}

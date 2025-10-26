// Primary Author: Mixed
package org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.TeliOp.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.teamcode.AAAOpModes.BaseOpMode;
@TeleOp(name="controllerTest")
public class ControllerTest extends BaseOpMode {
    @Override
    public void externalInit() {

    }
    public void externalLoop(){
        BaseOpMode.addData("x", driver1.cross.isPressed());
        BaseOpMode.addData("o", driver1.circle.isPressed());
        BaseOpMode.addData("triangle", driver1.triangle.isPressed());
        BaseOpMode.addData("square", driver1.square.isPressed());
        BaseOpMode.addData("dpadup",driver1.dpad_up.isPressed());
        BaseOpMode.addData("dpadleft",driver1.dpad_left.isPressed());
        BaseOpMode.addData("dpaddown",driver1.dpad_down.isPressed());
        BaseOpMode.addData("dpadright", driver1.dpad_right.isPressed());
        BaseOpMode.addData("options", driver1.options.isPressed());
        BaseOpMode.addData("rightBumper", driver1.rightBumper.isPressed());
        BaseOpMode.addData("leftBumpoer", driver1.leftBumper.isPressed());
        BaseOpMode.addData("rightTrigger", driver1.rightTrigger.isPressed());
        BaseOpMode.addData("leftTrigger", driver1.leftTrigger.isPressed());
        BaseOpMode.addData("ps", driver1.playstation.isPressed());
        BaseOpMode.addData("leftX", driver1.leftStick.X());
        BaseOpMode.addData("leftY", driver1.leftStick.Y());
        BaseOpMode.addData("RightX", driver1.rightStick.X());
        BaseOpMode.addData("RightY", driver1.rightStick.Y());
    }
}

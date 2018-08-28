
package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

/*
 * This is an example LinearOpMode that shows how to use
 * a legacy (NXT-compatible) Touch Sensor.
 * It assumes that the touch sensor is configured with a name of "sensor_touch".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name = "TouchTest", group = "Autonomous")
@Disabled
public class TouchTest extends LinearOpMode {

    TouchSensor button;  // Hardware Device Object

    @Override
    public void runOpMode() {

        // get a reference to our Light Sensor object.
        button = hardwareMap.touchSensor.get("button");
        int counter = 0;

        // wait for the start button to be pressed.
        waitForStart();

        // while the op mode is active, loop and read the light levels.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {

            // send the info back to driver station using telemetry function.
            if (button.isPressed())
                telemetry.addData("Touch", "Is Pressed");
            else
                telemetry.addData("Touch", "Is Not Pressed");

            telemetry.update();
        }
    }
}

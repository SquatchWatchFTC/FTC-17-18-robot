package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * Created by 8868 on 9/25/2017.
 */
@Autonomous( name = "jewel color reader", group = "Autonomous")
@Disabled
public class jewelColorReader extends LinearOpMode {

    private ColorSensor Color;
    private DcMotor leftMotor;
    private DcMotor rigtMotor;




    @Override
    public void runOpMode() {
        telemetry.addData("Status", "initialized");
        Color = (hardwareMap.colorSensor.get("color"));
        waitForStart();
        while (opModeIsActive()) {

            if (Color.blue() > Color.red()) {
                telemetry.addLine("its blue");
            } else if (Color.red() > Color.blue()) {
                telemetry.addLine("its red");
            } else {
                telemetry.addLine("indistinguishable");
            }
            telemetry.update();
        }
    }
}

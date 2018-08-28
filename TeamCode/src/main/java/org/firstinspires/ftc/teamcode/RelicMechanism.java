package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by 8868 on 9/7/2017.
 */
@TeleOp (name = "Relic_servo", group = "TeleOp")

//@Disabled

public class RelicMechanism extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private Servo grabber;
    private Servo lifter;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        grabber = (hardwareMap.servo.get("grabber"));
        lifter = (hardwareMap.servo.get("lifter"));
        telemetry.addData("version: ", "3.7 | 'All Our Base' Edition!");
        telemetry.update();

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        runtime.reset();

    }

    @Override
    public void loop() {

        if (gamepad1.a) {
            grabber.setPosition(1);
        } else {
            grabber.setPosition(.7);
        }
        if (gamepad1.y) {
            lifter.setPosition(.99);
        }else if (gamepad1.x) {
            lifter.setPosition(.49);
        }            else{
            lifter.setPosition(.1);
        }
    }
}



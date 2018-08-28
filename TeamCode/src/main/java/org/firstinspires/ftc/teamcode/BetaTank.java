package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by 8868 on 11/21/2017.
 */

@TeleOp (name = "BetaTank", group = "TeleOp")
public class BetaTank extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftmotor;
    private DcMotor rightmotor;
    private Servo servo;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        leftmotor = (hardwareMap.dcMotor.get("leftmotor"));
        rightmotor = (hardwareMap.dcMotor.get("rightmotor"));
        leftmotor.setDirection(DcMotor.Direction.REVERSE);

        servo = (hardwareMap.servo.get("Servo"));
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
        float leftY = gamepad1.left_stick_y;
        float rightY = gamepad1.right_stick_y;

        telemetry.addData("servo Pos" ,servo.getPosition());


        rightmotor.setPower(rightY);
        leftmotor.setPower(leftY);

        if (gamepad1.dpad_left){
            servo.setPosition(1);
        } else if(gamepad1.dpad_right){
            servo.setPosition(0);
        } else {
            servo.setPosition(.5);
        }
    }

    @Override
    public void stop() {
    }
}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by 8868 on 4/21/2018.
 */
@TeleOp (name = "dylan'sToyBot", group = "TeleOp")
//Disabled
public class DylansLittleBot extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor Leftmotor;
    private DcMotor Rightmotor;

    double power = 1;


    @Override
    public void init() {


        Leftmotor = (hardwareMap.dcMotor.get("Leftmotor"));
        Rightmotor = (hardwareMap.dcMotor.get("Rightmotor"));

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

        if (gamepad1.y && power < 1){
            power = power + .25;
        }
        if (gamepad1.a && power > 0){
            power = power - .25;
        }


        Leftmotor.setPower(leftY * power);
        Rightmotor.setPower(rightY * power);
    }
    @Override
    public void stop() {}
}
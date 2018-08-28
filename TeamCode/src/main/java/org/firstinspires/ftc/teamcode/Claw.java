package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.qualcomm.robotcore.hardware.Servo.MAX_POSITION;
import static com.qualcomm.robotcore.hardware.Servo.MIN_POSITION;

/**
 * Created by 8868 on 9/28/2017.
 */
@TeleOp (name="Claw", group="TeleOp")
@Disabled
public class Claw extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    Servo claw;

    @Override
    public void init(){
        telemetry.addData("Status", "Initialized");
        claw = hardwareMap.servo.get("claw");
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
            claw.setPosition(100);
        } else if (gamepad1.b){
            claw.setPosition(0);
        }
    }

    @Override
    public void stop() {
    }
}
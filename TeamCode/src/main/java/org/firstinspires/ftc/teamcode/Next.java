package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@TeleOp (name ="Next", group = "TeleOp")
@Disabled

public class Next extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor LeftMotor;
    private DcMotor RightMotor;
    private Servo Fan;


    @Override

    public void init(){
        telemetry.addData( "Stutus", "Initialized");

        LeftMotor = (hardwareMap.dcMotor.get("LeftMotor"));
        RightMotor = (hardwareMap.dcMotor.get("RightMotor"));
        Fan = (hardwareMap.servo.get("Fan"));


    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start () {
        runtime.reset();
    }

    @Override
    public void loop() {

        float leftY = gamepad1.left_stick_y;
        float rightY = gamepad1.right_stick_y;



        if (leftY > 0 && rightY < 0 || leftY < 0 && rightY > 0) {

            LeftMotor.setPower(leftY / 4);
            RightMotor.setPower(rightY / 4);
        }
        else {
            LeftMotor.setPower(leftY);
            RightMotor.setPower(rightY);
        }

        if (gamepad2.left_bumper) {

            Fan.setPosition(1);
        }
        else Fan.setPosition(0);  {

        }
    }

    @Override
    public void stop(){

    }
}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp (name = "RELIC_TEST", group = "TeleOp")

@Disabled

public class RELIC_TEST extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
   /* private DcMotor leftmotor;
    private DcMotor rightmotor;
    private DcMotor lift;
    private DcMotor slide;
    private DcMotor score;
    private DcMotor lights;
    private Servo arm;
    // private ColorSensor Color;
    private DcMotor rlift;
    private DcMotor rslide;
    private Servo rgrab;
    private Servo HJ;
    private Servo tss;

    boolean servoset = true;
*/
   private Servo relic;
   private Servo flip;
    @Override
    public void init(){
        telemetry.addData("Status", "Initialized");
/*
        leftmotor = (hardwareMap.dcMotor.get("leftmotor"));
        rightmotor = (hardwareMap.dcMotor.get("rightmotor"));
        leftmotor.setDirection(DcMotor.Direction.REVERSE);
        lift = (hardwareMap.dcMotor.get("lift"));
        slide = (hardwareMap.dcMotor.get("slide"));
        lights = (hardwareMap.dcMotor.get("lights"));
        arm = (hardwareMap.servo.get("arm"));
        rlift = (hardwareMap.dcMotor.get("rlift"));
        //rslide = (hardwareMap.dcMotor.get("rslide"));
        rgrab = (hardwareMap.servo.get("rgrab"));
        HJ = (hardwareMap.servo.get("hj"));
        tss = (hardwareMap.servo.get("tss"));
        score = (hardwareMap.dcMotor.get("score"));
*/
        flip = (hardwareMap.servo.get("flip"));
        relic = (hardwareMap.servo.get("relic"));
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
        telemetry.addData("version: ", "1.1.4 | Score edition for teleop");
        telemetry.update();
/*
        telemetry.addData("encodercount ", leftmotor.getCurrentPosition());
        telemetry.update();

        /*          Controller usage:

                  BUTTONS      |    GAMEPAD
                left stick     |        1
                right stick    |        1
                left trigger   |        1
                right trigger  |        1
                left bumper    |        1
                right bumper   |        1
                A              |        1
           --------------------|----------------
                A              |        2
                Y              |        2

         */
/*        float leftY = gamepad1.left_stick_y;
        float rightY = gamepad1.right_stick_y;
        float ltrig = gamepad1.left_trigger;
        float rtrig = gamepad1.right_trigger;
        float rightY_2 = gamepad2.right_stick_y;
        float leftY_2 = gamepad2.left_stick_y;
        arm.setPosition(.8);
        HJ.setPosition(.5);
        lights.setPower(.3);

        if (leftY_2 < .1 && leftY_2 > -.1 ){
            score.setPower(0);
        }
        else {
            score.setPower(leftY_2 / 2.2);
        }



        /*if (gamepad1.b) {
            rslide.setPower(1);}
        else if (gamepad1.x){
            rslide.setPower(-.2);}
        else{rslide.setPower(0);
        }*/
/*
        if (!gamepad2.dpad_down){
            tss.setPosition(0);
        }

        if (gamepad2.y){
            slide.setPower(1);

        }
        else if (gamepad2.a){
            slide.setPower(-.5);
        }
        else {
            slide.setPower(0);
        }

        if(gamepad2.b) {
            lift.setPower(-.3);
        }
        else if (gamepad2.x) {
            lift.setPower(.5);
        }
        else {lift.setPower(0);
        }

        if(gamepad1.dpad_left){
            rgrab.setPosition(1);}
        else if(gamepad1.dpad_right){
            rgrab.setPosition(0);
        }



        if(leftY > 0 && rightY < 0 || leftY < 0 && rightY > 0){
            leftmotor.setPower(leftY * .75);
            rightmotor.setPower(rightY * .75);
        }

        else if (leftY < .1 && leftY> -.1 && rightY < .1 && rightY > -.1){
            leftmotor.setPower(0);
            rightmotor.setPower(0);
        }

        else if (gamepad1.right_bumper){
            leftmotor.setPower(leftY / 2);
            rightmotor.setPower(rightY / 2);
        }


        else {
            leftmotor.setPower(leftY);
            rightmotor.setPower(rightY);
        }





  */  if (gamepad1.dpad_down){

      flip.setPosition(.9);
            }
            else if (gamepad1.dpad_left){
      flip.setPosition(.5);
        }
        else {flip.setPosition(.1);
    }
        if (gamepad1.a){
                relic.setPosition(.6);
        }
        else if (gamepad1.y){
            relic.setPosition(.9);
        }
            else relic.setPosition(.1);}
    @Override
    public void stop() {
    }
}
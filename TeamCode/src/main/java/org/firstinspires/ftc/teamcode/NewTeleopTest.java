package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by 8868 on 9/7/2017.
 */
@TeleOp (name = "WorldsTeleOp", group = "TeleOp")

//@Disabled

public class NewTeleopTest extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftRear;
    private DcMotor rightRear;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor GlyphWheels;
    //private DcMotor slide;
    private DcMotor score;
    //private DcMotor lights;
    private Servo arm;
   // private ColorSensor Color;
   // private DcMotor rlift;
   // private DcMotor rslide;
    private Servo rgrab;
    private Servo HJ;
    private Servo tss;
    //private Servo rbase;
    //private Servo ggrab;
    private Servo grabber;
    private Servo lifter;
    private DcMotor runner;

    double liftServo = 0;


    boolean servoset = true;
    boolean dPadInUse = false;
    //boolean relicSwitch = true;
    double speedValue = 0;
    double servoValue = .1;
    double rbaseServoValue = .2;

    @Override
    public void init(){
        telemetry.addData("Status", "Initialized");
        leftRear = (hardwareMap.dcMotor.get("leftRear"));
        rightRear = (hardwareMap.dcMotor.get("rightRear"));
        leftFront = (hardwareMap.dcMotor.get("leftFront"));
        rightFront = (hardwareMap.dcMotor.get("rightFront"));

        leftRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);

        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        GlyphWheels = (hardwareMap.dcMotor.get("lift"));
        //slide = (hardwareMap.dcMotor.get("slide"));
        //lights = (hardwareMap.dcMotor.get("lights"));
        arm = (hardwareMap.servo.get("arm"));
        //arm.setPosition(.9);
        //rlift = (hardwareMap.dcMotor.get("rlift"));
      //  rslide = (hardwareMap.dcMotor.get("rslide"));
      //  rgrab = (hardwareMap.servo.get("claw"));
        //rgrab.setPosition(.2);
        //rbase = hardwareMap.servo.get("rbase");
        HJ = (hardwareMap.servo.get("hj"));
        tss = (hardwareMap.servo.get("tss"));
       // ggrab = (hardwareMap.servo.get("ggrab"));
        //tss.setPosition(0);
        grabber = (hardwareMap.servo.get("grabber"));
        lifter = (hardwareMap.servo.get("lifter"));
        runner = (hardwareMap.dcMotor.get("runner"));
        score = (hardwareMap.dcMotor.get("score"));
        speedValue = 1;
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
        //rgrab.setPosition(.7);
        arm.setPosition(.9);
        HJ.setPosition(.62);
        //rgrab.setPosition(.3);
        float leftY = gamepad1.left_stick_y;
        float rightY = gamepad1.right_stick_y;
        float ltrig = gamepad1.left_trigger;
        float rtrig = gamepad1.right_trigger;
        float rightY_2 = gamepad2.right_stick_y;
        float leftY_2 = gamepad2.left_stick_y;

        /*if (gamepad2.a){
            ggrab.setPosition(.3);
        }
        else{
            ggrab.setPosition(0);
        }*/
        if (leftY_2 < .1 && leftY_2 > -.1) {
            score.setPower(0);
        } else {
            score.setPower(leftY_2 / 2.5);
        }

        if (gamepad1.right_bumper) {
            speedValue = .50;
        } else {
            speedValue = 1;
        }

        if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
            dPadInUse = true;
        } else {
            dPadInUse = false;
        }

        if (gamepad2.b) {
            GlyphWheels.setPower(-.3);
        } else if (gamepad2.x) {
            GlyphWheels.setPower(.7);
        } else {
            GlyphWheels.setPower(0);
        }

        if (dPadInUse) {
            if (gamepad1.dpad_left) {
                leftRear.setPower(-0.90);
                rightRear.setPower(0.90);
                leftFront.setPower(0.60);
                rightFront.setPower(-0.60);
            } else if (gamepad1.dpad_right) {
                leftRear.setPower(0.90);
                rightRear.setPower(-0.90);
                leftFront.setPower(-0.60);
                rightFront.setPower(0.60);
            } else {
                leftRear.setPower(0);
                rightRear.setPower(0);
                leftFront.setPower(0);
                rightFront.setPower(0);
            }

        } else {
            if (leftY < .1 && leftY > -.1 && rightY < .1 && rightY > -.1) {
                leftRear.setPower(0);
                rightRear.setPower(0);
                leftFront.setPower(0);
                rightFront.setPower(0);
            } else {
                leftRear.setPower(leftY * speedValue);
                rightRear.setPower(rightY * speedValue);
                leftFront.setPower(leftY * speedValue);
                rightFront.setPower(rightY * speedValue);
            }

            if (gamepad1.a && liftServo < .95) {
                liftServo = liftServo + 0.01;
            }
            if (gamepad1.x && liftServo > .05) {
                liftServo = liftServo - 0.01;
            }
            if (gamepad1.a && gamepad1.left_bumper){
                liftServo = liftServo + 0.002;
            }
            lifter.setPosition(liftServo);

            if (gamepad2.a) {

                grabber.setPosition(1);
            } else {
                grabber.setPosition(0);
            }
            if (gamepad1.y){

                runner.setPower(1);
            } else {
                runner.setPower(0);
            }
            if (gamepad1.left_bumper){
                runner.setPower(.4);
            }
            else {
                runner.setPower(0);
            }
            if (gamepad1.b){
                runner.setPower(-.3);
            } else{
                runner.setPower(0);
            }

        /**if (gamepad2.dpad_up) {
            if (rbaseServoValue < 0.95) {
                rbaseServoValue = rbaseServoValue + .01;
            } else {
                //don't do anything!
            }
        } else if (gamepad2.dpad_down) {
            if (rbaseServoValue > 0.05) {
                rbaseServoValue = rbaseServoValue - .01;
            } else {
                //don't do anything!
            }
        } else {
            //nothing to do
        }
        rbase.setPosition(rbaseServoValue);
*/

       /** if (gamepad1.x) {
            rslide.setPower(.5);
            telemetry.addLine("relic going out");
        } else if (gamepad1.b) {
            rslide.setPower(-1);
            telemetry.addLine("arm coming in");
        }   else {
            rslide.setPower(0);**/

/**
            if (gamepad1.a) {
                rgrab.setPosition(.1);
            }
            else {
                //do nothing
            }

*/

            //if (Color.blue() > Color.red()) {
            //telemetry.addLine("its blue");
            // } else if (Color.red() > Color.blue()) {
            // telemetry.addLine("its red");
            // }
            // telemetry.update();
            //pretending to code stuff so emily can get a decent picture

        }


    }@Override
    public void stop() {
    }
}
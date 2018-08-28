package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;




@TeleOp(name="teleoprobowa", group="teleop")
@Disabled
public class robot_for_brandon extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    int timesPassed;

    boolean isActive = false;
    boolean beaconIsActive = false;
    boolean killmyself = false;
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor sweeper;
    DcMotor shooter;
    DcMotor leftCannon;
    DcMotor rightCannon;
    DcMotor lift;
    Servo   leftServo;
    Servo   rightServo;
    Servo   stopper;

    DcMotor lights;
    DcMotor beacon;
    double timer = -1 ;
    double lastTime;






    public void encReset() throws InterruptedException {
        shooter.setMode(DcMotor.RunMode.RESET_ENCODERS);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    }
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");


        leftMotor = hardwareMap.dcMotor.get("leftmotor");
        rightMotor = hardwareMap.dcMotor.get("rightmotor");
        sweeper = hardwareMap.dcMotor.get("sweeper");
        shooter = hardwareMap.dcMotor.get("shooter");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftCannon = hardwareMap.dcMotor.get("leftCannon");
        rightCannon = hardwareMap.dcMotor.get("rightCannon");
        leftCannon.setDirection(DcMotor.Direction.REVERSE);
        leftServo = hardwareMap.servo.get("leftServo");
        rightServo = hardwareMap.servo.get("rightServo");
        lift = hardwareMap.dcMotor.get("lift");
        stopper = hardwareMap.servo.get("stopper");
        lights=hardwareMap.dcMotor.get("lights");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        beacon = hardwareMap.dcMotor.get("beacon");
        //beacon.setDirection(DcMotor.Direction.REVERSE);

        stopper.setPosition(0.5);
        rightServo.setPosition(1.0);
        leftServo.setPosition(0);


    }
    @Override
    public void init_loop() {
    }


    @Override
    public void start() {
        runtime.reset();
        runtime.startTime();
    }


    @Override
    public void loop() {



        float leftY = gamepad1.right_stick_y;
        float rightY = gamepad1.left_stick_y;
        float leftT = gamepad1.left_trigger;
        float rightT = gamepad1.right_trigger;
        lights.setPower(.5);

        if (gamepad1.left_bumper) {
            leftY = leftY * -1;
            rightY = rightY * -1;
        } else {
            //do stuffs
        }

        if (gamepad1.right_bumper) {
            leftMotor.setPower(leftY * 0.35);
            rightMotor.setPower(rightY * 0.35);
        } else {
            leftMotor.setPower(leftY);
            rightMotor.setPower(rightY);
        }

        if (rightT > 0) {
            sweeper.setPower(rightT);
        } else if (leftT > 0) {
            sweeper.setPower(leftT * -1);
        } else {
            sweeper.setPower(0);
        }

        if (gamepad1.a) {
            isActive = true;

        }

        if (gamepad2.x) {
            stopper.setPosition(0);
        } else {
            stopper.setPosition(0.5);
        }

        if (isActive) {
            if (shooter.getCurrentPosition() < 990) {
                telemetry.addData("Encoder Position: ", shooter.getCurrentPosition());
                shooter.setPower(0.5);
            } else {
                shooter.setPower(0);
                try {
                    encReset();
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                isActive = false;
            }
        }

        if (gamepad1.b) {
            leftCannon.setPower(1.0);
            rightCannon.setPower(1.0);
        } else if (gamepad2.y) {
            leftCannon.setPower(1);
            rightCannon.setPower(1);
        } else {
            leftCannon.setPower(0.0);
            rightCannon.setPower(0.0);
        }

        if (gamepad2.right_bumper) {
            lift.setPower(1);
            lastTime = runtime.milliseconds();
            killmyself = true;
        } else if (gamepad2.left_bumper) {
            lift.setPower(-.25);
            killmyself = false;
        } else {
            lift.setPower(0.0);
        }

        if (killmyself) {
            if (timer == -1 && runtime.milliseconds() >= lastTime + 2000) {
                timer = runtime.milliseconds();
            } else {
                if (timer <= runtime.milliseconds() + 2000) {
                    lights.setPower(1);
                } else {
                    lights.setPower(0);
                    timer = -1;
                    lastTime = runtime.milliseconds();
                }
            }
        }
        /*if (gamepad1.a) {
            shooter.setPower(1);
        } else if (gamepad1.x) {
            shooter.setPower(-.25);
        } else {
            shooter.setPower(0.0);
        }*/

        if (gamepad2.a) {
            leftServo.setPosition(1.0);
        } else {
            leftServo.setPosition(0.0);
        }

        if (gamepad2.a) {
            rightServo.setPosition(0.0);
        } else {
            rightServo.setPosition(1.0);
        }

        if (gamepad2.b) {
            beaconIsActive = true;
        }

        if (beaconIsActive) {
            if (gamepad2.b && beacon.getCurrentPosition() < 800) {
                beacon.setPower(0.5);
            } else if (gamepad2.b) {
                beacon.setPower(0);
            } else {
                if (beacon.getCurrentPosition() > 0) {
                    beacon.setPower(-0.25);
                } else {
                    beacon.setPower(0);
                    beaconIsActive = false;
                }
            }
        }

       /* if (killmyself) {
            if (gamepad1.a && shooter.getCurrentPosition() < 1600) {
                shooter.setPower(0.5);
            } else if (gamepad1.a) {
                shooter.setPower(0);
            } else {
                if (shooter.getCurrentPosition() > 0) {
                    shooter.setPower(-0.25);
                } else {
                    shooter.setPower(0);
                    killmyself = false;
                }
            }
        }

*/










    }









    @Override
    public void stop() {
    }
}

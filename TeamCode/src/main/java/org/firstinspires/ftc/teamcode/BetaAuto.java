package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by 8868 on 1/25/2018.
 */
@Autonomous (name = "BetaKnight", group = "Autonomous")

public class BetaAuto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftspin;
    private DcMotor rightspin;
    private DcMotor leftwheel;
    private DcMotor rightwheel;
    private ColorSensor color;
    private DcMotor blue;
    private DcMotor red;
    int balls;


    @Override
    public void runOpMode() throws InterruptedException {

        leftspin = (hardwareMap.dcMotor.get("leftspin"));
        rightspin = (hardwareMap.dcMotor.get("rightspin"));
        leftspin.setDirection(DcMotor.Direction.REVERSE);
        leftwheel = (hardwareMap.dcMotor.get("leftmotor"));
        rightwheel = (hardwareMap.dcMotor.get("rightmotor"));
        leftwheel.setDirection(DcMotor.Direction.REVERSE);
        color = (hardwareMap.colorSensor.get("color"));
        blue = (hardwareMap.dcMotor.get("blue light"));
        red = (hardwareMap.dcMotor.get("red light"));

        runtime.reset();
            while (balls < 4) {
                sleep(1000);
                if (color.red() > color.blue()) {
                    red.setPower(1);
                    blue.setPower(0);
                    leftwheel.setPower(-.15);
                    rightwheel.setPower(.15);
                    sleep(500);
                    leftwheel.setPower(0);
                    rightwheel.setPower(0);
                    sleep(200);
                    leftspin.setPower(1);
                    rightspin.setPower(1);
                    sleep(200);
                    leftspin.setPower(0);
                    rightspin.setPower(0);
                    leftwheel.setPower(.15);
                    rightwheel.setPower(-.15);
                    sleep(500);
                    leftwheel.setPower(0);
                    rightwheel.setPower(0);
                    balls++;
                }
                if (color.red() < color.blue()) {
                    blue.setPower(1);
                    red.setPower(0);
                    leftwheel.setPower(.15);
                    rightwheel.setPower(-.15);
                    sleep(500);
                    leftwheel.setPower(0);
                    rightwheel.setPower(0);
                    sleep(200);
                    leftspin.setPower(1);
                    rightspin.setPower(1);
                    sleep(200);
                    leftspin.setPower(0);
                    rightspin.setPower(0);
                    leftwheel.setPower(-.15);
                    rightwheel.setPower(.15);
                    sleep(500);
                    leftwheel.setPower(0);
                    rightwheel.setPower(0);
                    balls++;
                }
            }
        }
    }

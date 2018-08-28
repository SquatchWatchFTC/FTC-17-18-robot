package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by 8868 on 12/5/2017.
 */
@Autonomous (name = "Fuzion Red", group = "Autonomous")
@Disabled
public class Fuzion_Auto extends LinearOpMode {
    private DcMotor leftmotor;
    private  DcMotor rightmotor;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor arm;
    private Servo servol;
    private Servo servor;
    private DcMotor ramp;


    @Override
    public void runOpMode() throws InterruptedException {
        leftmotor = (hardwareMap.dcMotor.get("leftmotor"));
        rightmotor = (hardwareMap.dcMotor.get("rightmotor"));
        rightmotor.setDirection(DcMotor.Direction.REVERSE);
        arm = (hardwareMap.dcMotor.get("arm"));
        servol = (hardwareMap.servo.get("servol"));
        servor = (hardwareMap.servo.get("servor"));
        ramp = (hardwareMap.dcMotor.get("ramp"));

        waitForStart();
        runtime.reset();
        servol.setPosition(0);
        servor.setPosition(1);

        while (opModeIsActive()){

            sleep( 2000);
            leftmotor.setPower(-.5);
            rightmotor.setPower(-.5);
            sleep(2000);
            leftmotor.setPower(.25);
            rightmotor.setPower(-.25);
            sleep(1000);
            leftmotor.setPower(0);
            rightmotor.setPower(0);
            arm.setPower(-.5);
            sleep(2000);
            servor.setPosition(0);
            servol.setPosition(1);
            sleep(1000);
            arm.setPower(1);
            sleep(2000);
            arm.setPower(0);
            leftmotor.setPower(.5);
            rightmotor.setPower(.5);
            sleep(200);
            leftmotor.setPower(0);
            rightmotor.setPower(0);
        }
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by 8868 on 11/28/2017.
 */
@TeleOp(name="TeleOpForTests", group ="TeleOp")
@Disabled

public class TeleOpForTests extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftmotor;
    private DcMotor rightmotor;

    Orientation angles;

    private BNO055IMU imu;


    @Override
    public void init() {

        leftmotor = (hardwareMap.dcMotor.get("leftmotor"));
        rightmotor = (hardwareMap.dcMotor.get("rightmotor"));
        leftmotor.setDirection(DcMotor.Direction.REVERSE);

        BNO055IMU.Parameters param = new BNO055IMU.Parameters();
        param.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        param.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        param.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        param.loggingEnabled = true;
        param.loggingTag = "IMU";
        param.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(param);
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
        leftmotor.setPower(gamepad1.left_stick_y);
        rightmotor.setPower(gamepad1.right_stick_y);

        telemetry.addData("z", angles.firstAngle);
        telemetry.addData("y", angles.secondAngle);
        telemetry.addData("x", angles.thirdAngle);

        telemetry.update();
    }
    @Override
    public  void stop() {

    }
}
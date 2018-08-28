package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by 8868 on 11/30/2017.
 */
@Autonomous  (name="Gyro Test", group ="Autonomous")
//@Disabled
public class Gyro_Test extends LinearOpMode {

    //private DcMotor leftmotor;
    //private  DcMotor rightmotor;

    private BNO055IMU imu;

    //private DistanceSensor thirdGlyph;
    Orientation angles;
    DistanceSensor sensorDistance;

    boolean blockDetected = false;

    public void setBlockDetected(boolean newVal) { blockDetected = newVal; }

    Thread distanceUpdate = new Thread(new Runnable() {
        boolean detectedState = false;
        @Override
        public void run() {
            sensorDistance = hardwareMap.get(DistanceSensor.class, "thirdGlyph");
            while(opModeIsActive()) {
                if(sensorDistance.getDistance(DistanceUnit.CM) < 100) {
                    detectedState = true;
                } else {
                    detectedState = false;
                }
                setBlockDetected(detectedState);
            }
        }
    });


    @Override
    public void runOpMode() throws InterruptedException {

        //leftmotor = (hardwareMap.dcMotor.get("leftmotor"));
        //rightmotor = (hardwareMap.dcMotor.get("rightmotor"));
        //rightmotor.setDirection(DcMotor.Direction.REVERSE);
        //thirdGlyph = hardwareMap.get(DistanceSensor.class, "thirdGlyph");

        BNO055IMU.Parameters param = new BNO055IMU.Parameters();
        param.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        param.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        param.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        param.loggingEnabled = true;
        param.loggingTag = "IMU";
        param.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(param);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        waitForStart();
        distanceUpdate.start();

        while (opModeIsActive()) {


            telemetry.addData("first angle  x  ", angles.firstAngle);
            telemetry.addData("second angle  y ", angles.secondAngle);
            telemetry.addData("third angle  z  ", angles.thirdAngle);
            telemetry.addData("distance sensor", blockDetected);

            telemetry.update();

          /**  while (angles.secondAngle < 88 || angles.secondAngle > 92){
                leftmotor.setPower(.25);
                rightmotor.setPower(-.25);

                telemetry.update();
            } */
        }


    }
}

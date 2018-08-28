package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import java.lang.InterruptedException;
import java.util.Locale;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

import com.qualcomm.robotcore.hardware.DigitalChannel;

import javax.microedition.khronos.opengles.GL;
/**
 * Created by 8868 on 12/3/2017.
 */



/**
 FOR THOSE IN THE FUTURE THAT ARE TRYING TO UNDERSTAND WHAT THIS IS, GRTGT STANDS FOR:
 Something something Triple Glyph Test

 ...I know, its a stupid name.
 */

@Autonomous(name="Worlds_tripple glyph_red", group ="Autonomous")
//@Disabled
public class NotGonna_WorkR extends LinearOpMode {

public static final String TAG = "Vuforia VuMark Sample";

        OpenGLMatrix lastLocation = null;

        VuforiaLocalizer vuforia;
private ElapsedTime runtime = new ElapsedTime();
private double timeout;
private DcMotor leftRear;
private DcMotor rightRear;
private DcMotor leftFront;
private DcMotor rightFront;
private DcMotor GlyphWheels;
//private DcMotor GlyphSlide;
//  private DcMotor lights;
private ColorSensor Color;
private DistanceSensor thirdGlyph;
//private DistanceSensor Range;
private Servo arm;
private Servo Horizontal_jewel;
private Servo tss;
private Servo lifter;
private DigitalChannel redTouch;
private DcMotor score;

private int jewel = 0;
private boolean cryptobox;

private BNO055IMU imu;

        double wallDistance;

        boolean afterjewel = true;
        boolean Searching;
        boolean vuforiaHasRan = false;
        boolean blockDetected = false;

        Orientation angles;

public void encReset() throws InterruptedException {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

public void jewelRunner() throws InterruptedException {
        arm.setPosition(0.32);


        Horizontal_jewel.setPosition(.62);
        sleep(500);
        if (Color.blue() < Color.red()) {
        sleep(1000);
        Horizontal_jewel.setPosition(0.2); //Arm Motion
        sleep(500);
        arm.setPosition(0.50);
        sleep(500);
        Horizontal_jewel.setPosition(0.30);
        sleep(500);
        Horizontal_jewel.setPosition(.6);
        sleep(500);
        arm.setPosition(.98);
        } else if (Color.red() < Color.blue()) {
        sleep(1000);
        Horizontal_jewel.setPosition(1); //Arm Motion
        sleep(500);
        arm.setPosition(0.50);
        sleep(500);
        Horizontal_jewel.setPosition(0.70);
        sleep(500);
        Horizontal_jewel.setPosition(.56);
        sleep(500);
        arm.setPosition(.98);
        }
        }


        void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
@Override
public void run() {
        // Acquiring the angles is relatively expensive; we don't want
        // to do that in each of the three items that need that info, as that's
        // three times the necessary expense.
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        }
        });

        telemetry.addLine()
        .addData("status", new Func<String>() {
@Override
public String value() {
        return imu.getSystemStatus().toShortString();
        }
        })
        .addData("calib", new Func<String>() {
@Override
public String value() {
        return imu.getCalibrationStatus().toString();
        }
        });


        telemetry.addLine()
        .addData("first angle z", new Func<String>() {
@Override
public String value() {
        return formatAngle(angles.angleUnit, angles.firstAngle);
        }
        })
        .addData("second angle y", new Func<String>() {
@Override
public String value() {
        return formatAngle(angles.angleUnit, angles.secondAngle);
        }
        })
        .addData("third angle x", new Func<String>() {
@Override
public String value() {
        return formatAngle(angles.angleUnit, angles.thirdAngle);
        }
        });


        }

        String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
        }

        String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
        }

public void setBlockDetected(boolean newVal) {
        blockDetected = newVal;
        }

        Thread distanceUpdate = new Thread(new Runnable() {
        boolean detectedState = false;

@Override
public void run() {
        thirdGlyph = hardwareMap.get(DistanceSensor.class, "thirdGlyph");
        while (opModeIsActive()) {
        if (thirdGlyph.getDistance(DistanceUnit.CM) < 100) {
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
        leftFront = (hardwareMap.dcMotor.get("leftFront"));
        rightFront = (hardwareMap.dcMotor.get("rightFront"));
        leftRear = (hardwareMap.dcMotor.get("leftRear"));
        rightRear = (hardwareMap.dcMotor.get("rightRear"));
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        GlyphWheels = (hardwareMap.dcMotor.get("lift"));
        // GlyphSlide = (hardwareMap.dcMotor.get("slide"));
        //lights = (hardwareMap.dcMotor.get("lights"));
        arm = (hardwareMap.servo.get("arm"));
        score = (hardwareMap.dcMotor.get("score"));
        Horizontal_jewel = (hardwareMap.servo.get("hj"));
        Color = (hardwareMap.colorSensor.get("color"));
        tss = (hardwareMap.servo.get("tss"));
        lifter = (hardwareMap.servo.get("lifter"));
        redTouch = hardwareMap.get(DigitalChannel.class, "redTouch");
        redTouch.setMode(DigitalChannel.Mode.INPUT);

        //Range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");


        BNO055IMU.Parameters param = new BNO055IMU.Parameters();
        param.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        param.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        param.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        param.loggingEnabled = true;
        param.loggingTag = "IMU";
        param.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(param);

        composeTelemetry();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AeZDUmr/////AAAAGR0qx0W4b0DjrQzSeWTvM0913GjDHZlxXi0un1eAgTfrZ83+8A/O/aPH6q9qbOu1cbhXz2n6bRyBow3NyxH5hd2kYt699wAY0x7SEZ8OmmSjmL4GaA4aWl+Tp6NsSVPoW1aWWnya1eBvFW0BM3u+aA0gHrkD75yvsgIj20PWIulvAfEZ+9aYcuqllxDo2eg205JGDNmREw7s/zdEFqrCRdH0kF6bNr4W/o18AmHEy1M1T1obBiytthatrqS7cqHSJYWUp97kubLePID8qHk3q/jxrgWeu8FQ5A3rlUcdxqY4M/MtODp76jrcJ1rZqDBVpZxyvlI+FbHIJn9Tunuu2QyhEBezOtHAgUjQWk8EeQDP";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData("version: ", "2.0 |  Hopefully Working Left Program");
        telemetry.update();
        lifter.setPosition(0);
        waitForStart();

        distanceUpdate.start();

        runtime.reset();

        relicTrackables.activate();

        //while (opModeIsActive()) {

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        //10.16 cm

        jewelRunner();

        timeout = runtime.milliseconds();

        //this is where the vuforia checks are, it won't run any of it if 5 seconds pass from the timeout above.
        while (!vuforiaHasRan && opModeIsActive() && timeout + 5000 > runtime.milliseconds()) {
        vuMark = RelicRecoveryVuMark.from(relicTemplate);
        telemetry.addData("Timeout number: ", timeout + 5000);
        telemetry.addData("Current time: ", runtime.milliseconds());
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        if (vuMark.equals(RelicRecoveryVuMark.RIGHT)) {
        vuforiaHasRan = true;
        telemetry.addData("Read RIGHT ", 0);
        telemetry.update();
        while (!cryptobox && opModeIsActive()) {
        tss.setPosition(.4);
        while (redTouch.getState() && opModeIsActive()) {
        leftFront.setPower(-.21);
        leftRear.setPower(-.21);
        rightFront.setPower(-.26);
        rightRear.setPower(-.26);
        }
        cryptobox = true;

        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        tss.setPosition(0);

        sleep(1000);

                    /*leftmotor.setPower(0.20);
                    rightmotor.setPower(0.2);
                    while (Math.abs(leftmotor.getCurrentPosition()) < 100) {

                        telemetry.addData("Encoder Count", leftmotor.getCurrentPosition());
                        telemetry.update();
*/
        while (angles.firstAngle < 95 && opModeIsActive()) {

        //we figured out that telemetry.update() MUST BE USED to update
        //the value recieved from angles.firstAngle, otherwise the number
        //will never move
        //that is stupid, but its what needs to be done

                   /* telemetry.addData("current angle: ", angles.firstAngle);
                    telemetry.update();*/
        leftFront.setPower(-.275);
        leftRear.setPower(-.275);
        rightFront.setPower(.275);
        rightRear.setPower(.275);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        sleep(500);

        while (angles.firstAngle > 95 && opModeIsActive()) {
                   /* telemetry.addData("current angle: ", angles.firstAngle);
                    telemetry.update();*/
        leftFront.setPower(.28);
        leftRear.setPower(.28);
        rightFront.setPower(-.25);
        rightRear.setPower(-.25);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        sleep(1000);

        //commenting out the if becuase it may not need it with how close the turn is to begin with
        //and due to the fact that the gyro zeros on run, so it will be off if the robot is off on start
        //if (angles.firstAngle < 95 && angles.firstAngle > 85) {
        leftFront.setPower(.3);
        leftRear.setPower(.3);
        rightFront.setPower(.33);
        rightRear.setPower(.33);
        sleep(1350);
        GlyphWheels.setPower(-.6);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        sleep(1000);
        leftFront.setPower(-.25);
        leftRear.setPower(-.25);
        rightFront.setPower(-.25);
        rightRear.setPower(-.25);
        sleep(750);
        GlyphWheels.setPower(0);
        sleep(250);
                /*leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(.2);
                rightRear.setPower(.2);
                sleep(1200);
                leftFront.setPower(0.3);
                leftRear.setPower(.3);
                rightFront.setPower(0.32);
                rightRear.setPower(.32);
                sleep(2000);
                leftFront.setPower(-.25);
                leftRear.setPower(-.25);
                rightFront.setPower(-.25);
                rightRear.setPower(-.25);
                GlyphWheels.setPower(-.25);
                sleep(500);
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);
                GlyphWheels.setPower(0);*/

        while (angles.firstAngle > -45 && opModeIsActive()) {
        telemetry.addData("current angle: ", angles.firstAngle);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        leftFront.setPower(.6);//changed from .275
        leftRear.setPower(.6);
        rightFront.setPower(-.6);
        rightRear.setPower(-.6);
        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        sleep(500);

        //go to pile


        encReset();


        GlyphWheels.setPower(.6);
        leftRear.setPower(.6);
        leftFront.setPower(.6);
        rightFront.setPower(.6);
        rightRear.setPower(.6);
        while (Math.abs(leftFront.getCurrentPosition()) < 2500 && opModeIsActive()) {
        telemetry.addData("Encoder Count", leftFront.getCurrentPosition());
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if (blockDetected) {
        GlyphWheels.setPower(0);
        }
        }
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        encReset();

               /* while (angles.firstAngle < -55 && opModeIsActive()) {
                    telemetry.addData("current angle: ", angles.firstAngle);
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    leftFront.setPower(-.6);//changed from .275
                    leftRear.setPower(-.6);
                    rightFront.setPower(.6);
                    rightRear.setPower(.6);
                }

                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);
                sleep(500);


                GlyphWheels.setPower(1);
                leftFront.setPower(.6);
                leftRear.setPower(.6);
                rightFront.setPower(.6);
                rightRear.setPower(.6);
                while (Math.abs(leftFront.getCurrentPosition()) < 1000 && opModeIsActive()) {
                    telemetry.addData("Encoder Count", leftFront.getCurrentPosition());
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                }
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);
                encReset();
*/
                /*while (angles.firstAngle > -65 && opModeIsActive()) {
                    telemetry.addData("current angle: ", angles.firstAngle);
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    leftFront.setPower(.6);//changed from .275
                    leftRear.setPower(.6);
                    rightFront.setPower(-.6);
                    rightRear.setPower(-.6);
                }*/

              /*  GlyphWheels.setPower(.6);
                leftFront.setPower(.5); //.75
                leftRear.setPower(.5);
                rightFront.setPower(.7);
                rightRear.setPower(.7);
                while (Math.abs(leftFront.getCurrentPosition()) < 1500 && opModeIsActive()) {
                    telemetry.addData("Encoder Count", leftFront.getCurrentPosition());
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                }
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);
                encReset();

                leftFront.setPower(.7);
                leftRear.setPower(.7);
                rightFront.setPower(.5);
                rightRear.setPower(.5);
                while (Math.abs(leftFront.getCurrentPosition()) < 1000 && opModeIsActive()){
                    telemetry.addData("Encoder Count" , leftFront.getCurrentPosition());
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                }
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightRear.setPower(0);
                rightFront.setPower(0);*/

                /*while (angles.firstAngle > -80 && opModeIsActive()) {
                    telemetry.addData("current angle: ", angles.firstAngle);
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    leftFront.setPower(.6);//changed from .275
                    leftRear.setPower(.6);
                    rightFront.setPower(-.6);
                    rightRear.setPower(-.6);
                }

                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);
                sleep(500);

                encReset();
*/
        leftFront.setPower(-.3);
        leftRear.setPower(-.3);
        rightFront.setPower(-.3);
        rightRear.setPower(-.3);
        while (Math.abs(leftFront.getCurrentPosition()) < 700 && opModeIsActive()) {
        telemetry.addData("Encoder Count", leftFront.getCurrentPosition());
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if (blockDetected) {
        GlyphWheels.setPower(0);
        }
        }
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        //GlyphWheels.setPower(0);
        encReset();

        //GlyphWheels.setPower(.6);

        leftFront.setPower(.3);
        leftRear.setPower(.3);
        rightFront.setPower(-.3);
        rightRear.setPower(-.3);
        while (Math.abs(leftFront.getCurrentPosition()) < 500 && opModeIsActive()) {
        telemetry.addData("Encoder Count", leftFront.getCurrentPosition());
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if (blockDetected) {
        GlyphWheels.setPower(0);
        }
        }
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        //GlyphWheels.setPower(0);
        encReset();


        //GlyphWheels.setPower(.6);
        leftFront.setPower(.3);
        leftRear.setPower(.3);
        rightFront.setPower(.3);
        rightRear.setPower(.3);
        while (Math.abs(leftFront.getCurrentPosition()) < 1400 && opModeIsActive()) {
        telemetry.addData("Encoder Count", leftFront.getCurrentPosition());
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if (blockDetected) {
        GlyphWheels.setPower(0);
        }
        }
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        //GlyphWheels.setPower(0);
        encReset();


        //GlyphWheels.setPower(.6);

        leftFront.setPower(-.3);
        leftRear.setPower(-.3);
        rightFront.setPower(.3);
        rightRear.setPower(.3);
        while (Math.abs(leftFront.getCurrentPosition()) < 400 && opModeIsActive()) {
        telemetry.addData("Encoder Count", leftFront.getCurrentPosition());
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if (blockDetected) {
        GlyphWheels.setPower(0);
        }
        }
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        GlyphWheels.setPower(0);
        encReset();

              /*  GlyphWheels.setPower(.6);
                leftFront.setPower(-.3);
                leftRear.setPower(-.3);
                rightFront.setPower(.3);
                rightRear.setPower(.3);
                while (Math.abs(leftFront.getCurrentPosition()) < 300 && opModeIsActive()) {
                    telemetry.addData("Encoder Count", leftFront.getCurrentPosition());
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                }
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);
                GlyphWheels.setPower(0);
                encReset();

                GlyphWheels.setPower(.6);
                leftFront.setPower(.3);
                leftRear.setPower(.3);
                rightFront.setPower(.3);
                rightRear.setPower(.3);
                while (Math.abs(leftFront.getCurrentPosition()) < 1000 && opModeIsActive()) {
                    telemetry.addData("Encoder Count", leftFront.getCurrentPosition());
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                }
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);
                GlyphWheels.setPower(0);
                encReset();*/

        sleep(750);
        GlyphWheels.setPower(.7);
        encReset();
        leftFront.setPower(-.7); //.75
        leftRear.setPower(-.7);
        rightFront.setPower(-.7);
        rightRear.setPower(-.7);
        while (Math.abs(leftFront.getCurrentPosition()) < 2200 && opModeIsActive()) {
        telemetry.addData("Encoder Count", leftFront.getCurrentPosition());
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        GlyphWheels.setPower(0);
        encReset();

                /*GlyphWheels.setPower(.4);
                sleep(1000);;
                score.setPower(.3);
                sleep(500);
                score.setPower(.4);
                sleep(1500);
                score.setPower(-.25);
                sleep(1000);

                leftFront.setPower(.5); //.75
                leftRear.setPower(.5);
                rightFront.setPower(.48);
                rightRear.setPower(.48);
                while (Math.abs(leftFront.getCurrentPosition()) < 300 && opModeIsActive()) {
                    telemetry.addData("Encoder Count", leftFront.getCurrentPosition());
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                }
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);
                encReset();*/
        leftRear.setPower(-.25);
        leftFront.setPower(-.25);
        rightRear.setPower(-.25);
        rightFront.setPower(-.25);
        sleep(1500);
        leftFront.setPower(.25); //.75
        leftRear.setPower(.25);
        rightFront.setPower(.25);
        rightRear.setPower(.25);
        while (Math.abs(leftFront.getCurrentPosition()) < 300 && opModeIsActive()) {
        telemetry.addData("Encoder Count", leftFront.getCurrentPosition());
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }
        leftFront.setPower(0); //.75
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);


        } else if (vuMark.equals(RelicRecoveryVuMark.CENTER) && timeout + 5000 > runtime.milliseconds()) {
        vuforiaHasRan = true;
        telemetry.addData("Read CENTER ", 0);
        telemetry.update();

        while (!cryptobox && opModeIsActive() || timeout + 1000 > runtime.milliseconds()) {
        tss.setPosition(.4);
        while (redTouch.getState() && opModeIsActive()) {
        leftFront.setPower(-.21);
        leftRear.setPower(-.21);
        rightFront.setPower(-.26);
        rightRear.setPower(-.26);

        }
        cryptobox = true;
        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        tss.setPosition(0);
        encReset();
        sleep(1000);


        leftFront.setPower(-0.20);
        leftRear.setPower(-0.20);
        rightFront.setPower(-.20);
        rightRear.setPower(-0.20);
        while (Math.abs(leftFront.getCurrentPosition()) < 875 && opModeIsActive()) {
        telemetry.addData("Encoder Count", leftFront.getCurrentPosition());
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        encReset();
        while (angles.firstAngle < 100 && opModeIsActive()) {

        //we figured out that telemetry.update() MUST BE USED to update
        //the value recieved from angles.firstAngle, otherwise the number
        //will never move
        //that is stupid, but its what needs to be done
        telemetry.addData("current angle: ", angles.firstAngle);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        leftFront.setPower(-.275);
        leftRear.setPower(-.275);
        rightFront.setPower(.275);
        rightRear.setPower(.275);
        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        sleep(500);

        while (angles.firstAngle > 100 && opModeIsActive()) {
        telemetry.addData("current angle: ", angles.firstAngle);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        leftFront.setPower(.22);
        leftRear.setPower(.22);
        rightFront.setPower(-.24);
        rightRear.setPower(-.24);
        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        sleep(1000);

        //commenting out the if becuase it may not need it with how close the turn is to begin with
        //and due to the fact that the gyro zeros on run, so it will be off if the robot is off on start
        //if (angles.firstAngle < 95 && angles.firstAngle > 85) {
        leftFront.setPower(.28);
        leftRear.setPower(.28);
        rightFront.setPower(.32);
        rightRear.setPower(.32);
        sleep(1500);
        GlyphWheels.setPower(-1);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        sleep(1000);
        leftFront.setPower(-.25);
        leftRear.setPower(-.25);
        rightFront.setPower(-.25);
        rightRear.setPower(-.25);
        sleep(750);
        GlyphWheels.setPower(0);
        sleep(250);
                /*leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(.2);
                rightRear.setPower(.2);
                sleep(1200);
                leftFront.setPower(0.27);
                leftRear.setPower(.27);
                rightFront.setPower(.31);
                rightRear.setPower(0.31);
                sleep(2000);
                leftFront.setPower(-.25);
                leftRear.setPower(-.25);
                rightFront.setPower(-.25);
                rightRear.setPower(-.25);
                GlyphWheels.setPower(-.25);
                sleep(1000);
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);
                GlyphWheels.setPower(0);*/

        while (angles.firstAngle > -45 && opModeIsActive()) {
        telemetry.addData("current angle: ", angles.firstAngle);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if (blockDetected) {
        GlyphWheels.setPower(0);
        }
        leftFront.setPower(.6);//changed from .275
        leftRear.setPower(.6);
        rightFront.setPower(-.6);
        rightRear.setPower(-.6);
        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        sleep(500);

        //go to pile


        encReset();


        GlyphWheels.setPower(.6);
        leftRear.setPower(.6);
        leftFront.setPower(.6);
        rightFront.setPower(.6);
        rightRear.setPower(.6);
        while (Math.abs(leftFront.getCurrentPosition()) < 2300 && opModeIsActive()) {
        telemetry.addData("Encoder Count", leftFront.getCurrentPosition());
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if (blockDetected) {
        GlyphWheels.setPower(0);
        }
        }
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        encReset();

               /* while (angles.firstAngle < -55 && opModeIsActive()) {
                    telemetry.addData("current angle: ", angles.firstAngle);
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    leftFront.setPower(-.6);//changed from .275
                    leftRear.setPower(-.6);
                    rightFront.setPower(.6);
                    rightRear.setPower(.6);
                }

                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);
                sleep(500);


                GlyphWheels.setPower(1);
                leftFront.setPower(.6);
                leftRear.setPower(.6);
                rightFront.setPower(.6);
                rightRear.setPower(.6);
                while (Math.abs(leftFront.getCurrentPosition()) < 1000 && opModeIsActive()) {
                    telemetry.addData("Encoder Count", leftFront.getCurrentPosition());
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                }
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);
                encReset();
*/
                /*while (angles.firstAngle > -65 && opModeIsActive()) {
                    telemetry.addData("current angle: ", angles.firstAngle);
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    leftFront.setPower(.6);//changed from .275
                    leftRear.setPower(.6);
                    rightFront.setPower(-.6);
                    rightRear.setPower(-.6);
                }*/

              /*  GlyphWheels.setPower(.6);
                leftFront.setPower(.5); //.75
                leftRear.setPower(.5);
                rightFront.setPower(.7);
                rightRear.setPower(.7);
                while (Math.abs(leftFront.getCurrentPosition()) < 1500 && opModeIsActive()) {
                    telemetry.addData("Encoder Count", leftFront.getCurrentPosition());
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                }
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);
                encReset();

                leftFront.setPower(.7);
                leftRear.setPower(.7);
                rightFront.setPower(.5);
                rightRear.setPower(.5);
                while (Math.abs(leftFront.getCurrentPosition()) < 1000 && opModeIsActive()){
                    telemetry.addData("Encoder Count" , leftFront.getCurrentPosition());
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                }
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightRear.setPower(0);
                rightFront.setPower(0);*/

                /*while (angles.firstAngle > -80 && opModeIsActive()) {
                    telemetry.addData("current angle: ", angles.firstAngle);
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    leftFront.setPower(.6);//changed from .275
                    leftRear.setPower(.6);
                    rightFront.setPower(-.6);
                    rightRear.setPower(-.6);
                }

                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);
                sleep(500);

                encReset();
*/
        leftFront.setPower(-.3);
        leftRear.setPower(-.3);
        rightFront.setPower(-.3);
        rightRear.setPower(-.3);
        while (Math.abs(leftFront.getCurrentPosition()) < 700 && opModeIsActive()) {
        telemetry.addData("Encoder Count", leftFront.getCurrentPosition());
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if (blockDetected) {
        GlyphWheels.setPower(0);
        }
        }
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        //GlyphWheels.setPower(0);
        encReset();

        //GlyphWheels.setPower(.6);
        leftFront.setPower(.3);
        leftRear.setPower(.3);
        rightFront.setPower(-.3);
        rightRear.setPower(-.3);
        while (Math.abs(leftFront.getCurrentPosition()) < 500 && opModeIsActive()) {
        telemetry.addData("Encoder Count", leftFront.getCurrentPosition());
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if (blockDetected) {
        GlyphWheels.setPower(0);
        }
        }
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        //GlyphWheels.setPower(0);
        encReset();


        //GlyphWheels.setPower(.6);
        leftFront.setPower(.3);
        leftRear.setPower(.3);
        rightFront.setPower(.3);
        rightRear.setPower(.3);
        while (Math.abs(leftFront.getCurrentPosition()) < 1400 && opModeIsActive()) {
        telemetry.addData("Encoder Count", leftFront.getCurrentPosition());
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if (blockDetected) {
        GlyphWheels.setPower(0);
        }
        }
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        //GlyphWheels.setPower(0);
        encReset();


        //GlyphWheels.setPower(.6);
        leftFront.setPower(-.3);
        leftRear.setPower(-.3);
        rightFront.setPower(.3);
        rightRear.setPower(.3);
        while (Math.abs(leftFront.getCurrentPosition()) < 300 && opModeIsActive()) {
        telemetry.addData("Encoder Count", leftFront.getCurrentPosition());
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if (blockDetected) {
        GlyphWheels.setPower(0);
        }
        }
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        GlyphWheels.setPower(0);
        encReset();

              /*  GlyphWheels.setPower(.6);
                leftFront.setPower(-.3);
                leftRear.setPower(-.3);
                rightFront.setPower(.3);
                rightRear.setPower(.3);
                while (Math.abs(leftFront.getCurrentPosition()) < 300 && opModeIsActive()) {
                    telemetry.addData("Encoder Count", leftFront.getCurrentPosition());
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                }
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);
                GlyphWheels.setPower(0);
                encReset();

                GlyphWheels.setPower(.6);
                leftFront.setPower(.3);
                leftRear.setPower(.3);
                rightFront.setPower(.3);
                rightRear.setPower(.3);
                while (Math.abs(leftFront.getCurrentPosition()) < 1000 && opModeIsActive()) {
                    telemetry.addData("Encoder Count", leftFront.getCurrentPosition());
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                }
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);
                GlyphWheels.setPower(0);
                encReset();*/
        sleep(750);
        GlyphWheels.setPower(.7);
        encReset();
        leftFront.setPower(-.9); //.75
        leftRear.setPower(-.9);
        rightFront.setPower(-.9);
        rightRear.setPower(-.9);
        while (Math.abs(leftFront.getCurrentPosition()) < 1800 && opModeIsActive()) {
        telemetry.addData("Encoder Count", leftFront.getCurrentPosition());
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        GlyphWheels.setPower(0);
        encReset();

               /* GlyphWheels.setPower(.4);
                sleep(1000);;
                score.setPower(.4);
                sleep(1500);*/
        leftRear.setPower(-.25);
        leftFront.setPower(-.25);
        rightRear.setPower(-.25);
        rightFront.setPower(-.25);
        sleep(1500);
        leftRear.setPower(.25);
        leftFront.setPower(.25);
        rightRear.setPower(.25);
        rightFront.setPower(.25);
        sleep(750);

                /*score.setPower(-.25);
                sleep(1000);

                leftFront.setPower(.5); //.75
                leftRear.setPower(.5);
                rightFront.setPower(.48);
                rightRear.setPower(.48);
                while (Math.abs(leftFront.getCurrentPosition()) < 300 && opModeIsActive()) {
                    telemetry.addData("Encoder Count", leftFront.getCurrentPosition());
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                }
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);
                encReset();*/


        } else if (vuMark.equals(RelicRecoveryVuMark.LEFT) && timeout + 5000 > runtime.milliseconds()) {
        vuforiaHasRan = true;
        telemetry.addData("Read LEFT ", 0);
        telemetry.update();

        while (!cryptobox && opModeIsActive() || timeout + 1000 > runtime.milliseconds()) {
        tss.setPosition(.5);
        while (redTouch.getState() && opModeIsActive()) {
        leftFront.setPower(-.23);
        leftRear.setPower(-.23);
        rightFront.setPower(-.28);
        rightRear.setPower(-.28);
        //lights.setPower(.2);
        }
        cryptobox = true;
        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        tss.setPosition(0);
        encReset();
        sleep(1000);
        encReset();
        leftFront.setPower(-0.20);
        leftRear.setPower(-.2);
        rightFront.setPower(-0.2);
        rightRear.setPower(-.2);
        while (Math.abs(leftFront.getCurrentPosition()) < 1300 && opModeIsActive()) {
        telemetry.addData("Encoder Count", leftFront.getCurrentPosition());
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        encReset();
        while (angles.firstAngle < 100 && opModeIsActive()) {

        //we figured out that telemetry.update() MUST BE USED to update
        //the value recieved from angles.firstAngle, otherwise the number
        //will never move
        //that is stupid, but its what needs to be done
        telemetry.addData("current angle: ", angles.firstAngle);
        telemetry.update();
        leftFront.setPower(-.275);
        leftRear.setPower(-.275);
        rightFront.setPower(.275);
        rightRear.setPower(.275);
        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        sleep(500);

        while (angles.firstAngle > 100 && opModeIsActive()) {
        telemetry.addData("current angle: ", angles.firstAngle);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        leftFront.setPower(.22);
        leftRear.setPower(.22);
        rightFront.setPower(-.24);
        rightRear.setPower(-.24);
        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        sleep(1000);

        //commenting out the if becuase it may not need it with how close the turn is to begin with
        //and due to the fact that the gyro zeros on run, so it will be off if the robot is off on start
        //if (angles.firstAngle < 95 && angles.firstAngle > 85) {
        leftFront.setPower(.25);
        leftRear.setPower(.25);
        rightFront.setPower(.3);
        rightRear.setPower(.3);
        sleep(750);
        GlyphWheels.setPower(-1);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        sleep(1000);
        leftFront.setPower(-0.25);
        leftRear.setPower(-.25);
        rightFront.setPower(-0.25);
        rightRear.setPower(-.25);
        while (Math.abs(leftFront.getCurrentPosition()) < 400 && opModeIsActive()) {
        telemetry.addData("Encoder Count", leftFront.getCurrentPosition());
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        encReset();


        while (angles.firstAngle > -65 && opModeIsActive()) {
        telemetry.addData("current angle: ", angles.firstAngle);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        leftFront.setPower(.6);//changed from .275
        leftRear.setPower(.6);
        rightFront.setPower(-.6);
        rightRear.setPower(-.6);
        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        sleep(500);

        //go to pile


        encReset();


        /**GlyphWheels.setPower(.6);
         leftRear.setPower(.6);
         leftFront.setPower(.6);
         rightFront.setPower(.6);
         rightRear.setPower(.6);
         while (Math.abs(leftFront.getCurrentPosition()) < 2300 && opModeIsActive()) {
         telemetry.addData("Encoder Count", leftFront.getCurrentPosition());
         angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
         if (blockDetected) {
         GlyphWheels.setPower(0);
         }
         }
         leftFront.setPower(0);
         leftRear.setPower(0);
         rightFront.setPower(0);
         rightRear.setPower(0);
         encReset();

         /* while (angles.firstAngle < -55 && opModeIsActive()) {
         telemetry.addData("current angle: ", angles.firstAngle);
         angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
         leftFront.setPower(-.6);//changed from .275
         leftRear.setPower(-.6);
         rightFront.setPower(.6);
         rightRear.setPower(.6);
         }

         leftFront.setPower(0);
         leftRear.setPower(0);
         rightFront.setPower(0);
         rightRear.setPower(0);
         sleep(500);


         GlyphWheels.setPower(1);
         leftFront.setPower(.6);
         leftRear.setPower(.6);
         rightFront.setPower(.6);
         rightRear.setPower(.6);
         while (Math.abs(leftFront.getCurrentPosition()) < 1000 && opModeIsActive()) {
         telemetry.addData("Encoder Count", leftFront.getCurrentPosition());
         angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
         }
         leftFront.setPower(0);
         leftRear.setPower(0);
         rightFront.setPower(0);
         rightRear.setPower(0);
         encReset();

         while (angles.firstAngle > -65 && opModeIsActive()) {
         telemetry.addData("current angle: ", angles.firstAngle);
         angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
         leftFront.setPower(.6);//changed from .275
         leftRear.setPower(.6);
         rightFront.setPower(-.6);
         rightRear.setPower(-.6);
         }

         GlyphWheels.setPower(.6);
         leftFront.setPower(.5); //.75
         leftRear.setPower(.5);
         rightFront.setPower(.7);
         rightRear.setPower(.7);
         while (Math.abs(leftFront.getCurrentPosition()) < 1500 && opModeIsActive()) {
         telemetry.addData("Encoder Count", leftFront.getCurrentPosition());
         angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
         }
         leftFront.setPower(0);
         leftRear.setPower(0);
         rightFront.setPower(0);
         rightRear.setPower(0);
         encReset();

         leftFront.setPower(.7);
         leftRear.setPower(.7);
         rightFront.setPower(.5);
         rightRear.setPower(.5);
         while (Math.abs(leftFront.getCurrentPosition()) < 1000 && opModeIsActive()){
         telemetry.addData("Encoder Count" , leftFront.getCurrentPosition());
         angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
         }
         leftFront.setPower(0);
         leftRear.setPower(0);
         rightRear.setPower(0);
         rightFront.setPower(0);

         /*while (angles.firstAngle > -80 && opModeIsActive()) {
         telemetry.addData("current angle: ", angles.firstAngle);
         angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
         leftFront.setPower(.6);//changed from .275
         leftRear.setPower(.6);
         rightFront.setPower(-.6);
         rightRear.setPower(-.6);
         }

         leftFront.setPower(0);
         leftRear.setPower(0);
         rightFront.setPower(0);
         rightRear.setPower(0);
         sleep(500);

         encReset();

         leftFront.setPower(-.3);
         leftRear.setPower(-.3);
         rightFront.setPower(-.3);
         rightRear.setPower(-.3);
         while (Math.abs(leftFront.getCurrentPosition()) < 700 && opModeIsActive()) {
         telemetry.addData("Encoder Count", leftFront.getCurrentPosition());
         angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
         if (blockDetected) {
         GlyphWheels.setPower(0);
         }
         }
         leftFront.setPower(0);
         leftRear.setPower(0);
         rightFront.setPower(0);
         rightRear.setPower(0);
         //GlyphWheels.setPower(0);
         encReset();

         //GlyphWheels.setPower(.6);
         leftFront.setPower(-.3);
         leftRear.setPower(-.3);
         rightFront.setPower(.3);
         rightRear.setPower(.3);
         while (Math.abs(leftFront.getCurrentPosition()) < 500 && opModeIsActive()) {
         telemetry.addData("Encoder Count", leftFront.getCurrentPosition());
         angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
         if (blockDetected) {
         GlyphWheels.setPower(0);
         }
         }
         leftFront.setPower(0);
         leftRear.setPower(0);
         rightFront.setPower(0);
         rightRear.setPower(0);
         //GlyphWheels.setPower(0);
         encReset();


         //GlyphWheels.setPower(.6);
         leftFront.setPower(.3);
         leftRear.setPower(.3);
         rightFront.setPower(.3);
         rightRear.setPower(.3);
         while (Math.abs(leftFront.getCurrentPosition()) < 1400 && opModeIsActive()) {
         telemetry.addData("Encoder Count", leftFront.getCurrentPosition());
         angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
         if (blockDetected) {
         GlyphWheels.setPower(0);
         }
         }
         leftFront.setPower(0);
         leftRear.setPower(0);
         rightFront.setPower(0);
         rightRear.setPower(0);
         //GlyphWheels.setPower(0);
         encReset();


         //GlyphWheels.setPower(.6);
         leftFront.setPower(.3);
         leftRear.setPower(.3);
         rightFront.setPower(-.3);
         rightRear.setPower(-.3);
         while (Math.abs(leftFront.getCurrentPosition()) < 500 && opModeIsActive()) {
         telemetry.addData("Encoder Count", leftFront.getCurrentPosition());
         angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
         if (blockDetected) {
         GlyphWheels.setPower(0);
         }
         }
         leftFront.setPower(0);
         leftRear.setPower(0);
         rightFront.setPower(0);
         rightRear.setPower(0);
         GlyphWheels.setPower(0);
         encReset();

         GlyphWheels.setPower(.6);
         leftFront.setPower(-.3);
         leftRear.setPower(-.3);
         rightFront.setPower(.3);
         rightRear.setPower(.3);
         while (Math.abs(leftFront.getCurrentPosition()) < 300 && opModeIsActive()) {
         telemetry.addData("Encoder Count", leftFront.getCurrentPosition());
         angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
         }
         leftFront.setPower(0);
         leftRear.setPower(0);
         rightFront.setPower(0);
         rightRear.setPower(0);
         GlyphWheels.setPower(0);
         encReset();

         GlyphWheels.setPower(.6);
         leftFront.setPower(.3);
         leftRear.setPower(.3);
         rightFront.setPower(.3);
         rightRear.setPower(.3);
         while (Math.abs(leftFront.getCurrentPosition()) < 1000 && opModeIsActive()) {
         telemetry.addData("Encoder Count", leftFront.getCurrentPosition());
         angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
         }
         leftFront.setPower(0);
         leftRear.setPower(0);
         rightFront.setPower(0);
         rightRear.setPower(0);
         GlyphWheels.setPower(0);
         encReset();
         sleep(750);
         GlyphWheels.setPower(.7);
         encReset();
         leftFront.setPower(-.9); //.75
         leftRear.setPower(-.9);
         rightFront.setPower(-.9);
         rightRear.setPower(-.9);
         while (Math.abs(leftFront.getCurrentPosition()) < 1800 && opModeIsActive()) {
         telemetry.addData("Encoder Count", leftFront.getCurrentPosition());
         angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
         }
         leftFront.setPower(0);
         leftRear.setPower(0);
         rightFront.setPower(0);
         rightRear.setPower(0);
         GlyphWheels.setPower(0);
         encReset();

         GlyphWheels.setPower(.4);
         sleep(1000);;
         score.setPower(.4);
         sleep(1500);
         leftRear.setPower(-.25);
         leftFront.setPower(-.25);
         rightRear.setPower(-.25);
         rightFront.setPower(-.25);
         sleep(1500);
         leftRear.setPower(.25);
         leftFront.setPower(.25);
         rightRear.setPower(.25);
         rightFront.setPower(.25);
         sleep(750);**/
        }
        if (!vuforiaHasRan) {
        telemetry.addData("Nothing read, TIMEOUT ", 0);
        telemetry.update();
        while (!cryptobox && opModeIsActive()) {
        tss.setPosition(.4);
        while (redTouch.getState() && opModeIsActive()) {
        leftFront.setPower(-.21);
        leftRear.setPower(-.21);
        rightFront.setPower(-.26);
        rightRear.setPower(-.26);
        }
        cryptobox = true;

        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        tss.setPosition(0);

        sleep(1000);

                    /*leftmotor.setPower(0.20);
                    rightmotor.setPower(0.2);
                    while (Math.abs(leftmotor.getCurrentPosition()) < 100) {

                        telemetry.addData("Encoder Count", leftmotor.getCurrentPosition());
                        telemetry.update();
*/
        while (angles.firstAngle < 95 && opModeIsActive()) {

        //we figured out that telemetry.update() MUST BE USED to update
        //the value recieved from angles.firstAngle, otherwise the number
        //will never move
        //that is stupid, but its what needs to be done

                   /* telemetry.addData("current angle: ", angles.firstAngle);
                    telemetry.update();*/
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        leftFront.setPower(-.275);
        leftRear.setPower(-.275);
        rightFront.setPower(.275);
        rightRear.setPower(.275);
        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        sleep(500);

        while (angles.firstAngle > 95 && opModeIsActive()) {
                    /*telemetry.addData("current angle: ", angles.firstAngle);
                    telemetry.update();*/
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        leftFront.setPower(.28);
        leftRear.setPower(.28);
        rightFront.setPower(-.25);
        rightRear.setPower(-.25);
        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        sleep(1000);

        //commenting out the if becuase it may not need it with how close the turn is to begin with
        //and due to the fact that the gyro zeros on run, so it will be off if the robot is off on start
        //if (angles.firstAngle < 95 && angles.firstAngle > 85) {
        leftFront.setPower(.3);
        leftRear.setPower(.3);
        rightFront.setPower(.33);
        rightRear.setPower(.33);
        sleep(1350);
        GlyphWheels.setPower(-.3);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        sleep(1000);
        leftFront.setPower(-.25);
        leftRear.setPower(-.25);
        rightFront.setPower(-.25);
        rightRear.setPower(-.25);
        sleep(500);
        GlyphWheels.setPower(0);
                 /*leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(.2);
                rightRear.setPower(.2);
                sleep(1200);
                leftFront.setPower(0.3);
                leftRear.setPower(.3);
                rightFront.setPower(0.32);
                rightRear.setPower(.32);
                sleep(2000);
                leftFront.setPower(-.25);
                leftRear.setPower(-.25);
                rightFront.setPower(-.25);
                rightRear.setPower(-.25);
                GlyphWheels.setPower(-.25);
                sleep(500);
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);
                GlyphWheels.setPower(0);*/

        while (angles.firstAngle > -45 && opModeIsActive()) {
        telemetry.addData("current angle: ", angles.firstAngle);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        leftFront.setPower(.6);//changed from .275
        leftRear.setPower(.6);
        rightFront.setPower(-.6);
        rightRear.setPower(-.6);
        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        sleep(500);

        //go to pile


        encReset();


        GlyphWheels.setPower(.6);
        leftRear.setPower(.6);
        leftFront.setPower(.6);
        rightFront.setPower(.6);
        rightRear.setPower(.6);
        while (Math.abs(leftFront.getCurrentPosition()) < 2500 && opModeIsActive()) {
        telemetry.addData("Encoder Count", leftFront.getCurrentPosition());
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if (blockDetected) {
        GlyphWheels.setPower(0);
        }
        }
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        encReset();

               /* while (angles.firstAngle < -55 && opModeIsActive()) {
                    telemetry.addData("current angle: ", angles.firstAngle);
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    leftFront.setPower(-.6);//changed from .275
                    leftRear.setPower(-.6);
                    rightFront.setPower(.6);
                    rightRear.setPower(.6);
                }

                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);
                sleep(500);


                GlyphWheels.setPower(1);
                leftFront.setPower(.6);
                leftRear.setPower(.6);
                rightFront.setPower(.6);
                rightRear.setPower(.6);
                while (Math.abs(leftFront.getCurrentPosition()) < 1000 && opModeIsActive()) {
                    telemetry.addData("Encoder Count", leftFront.getCurrentPosition());
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                }
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);
                encReset();
*/
                /*while (angles.firstAngle > -65 && opModeIsActive()) {
                    telemetry.addData("current angle: ", angles.firstAngle);
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    leftFront.setPower(.6);//changed from .275
                    leftRear.setPower(.6);
                    rightFront.setPower(-.6);
                    rightRear.setPower(-.6);
                }*/

              /*  GlyphWheels.setPower(.6);
                leftFront.setPower(.5); //.75
                leftRear.setPower(.5);
                rightFront.setPower(.7);
                rightRear.setPower(.7);
                while (Math.abs(leftFront.getCurrentPosition()) < 1500 && opModeIsActive()) {
                    telemetry.addData("Encoder Count", leftFront.getCurrentPosition());
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                }
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);
                encReset();

                leftFront.setPower(.7);
                leftRear.setPower(.7);
                rightFront.setPower(.5);
                rightRear.setPower(.5);
                while (Math.abs(leftFront.getCurrentPosition()) < 1000 && opModeIsActive()){
                    telemetry.addData("Encoder Count" , leftFront.getCurrentPosition());
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                }
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightRear.setPower(0);
                rightFront.setPower(0);*/

                /*while (angles.firstAngle > -80 && opModeIsActive()) {
                    telemetry.addData("current angle: ", angles.firstAngle);
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    leftFront.setPower(.6);//changed from .275
                    leftRear.setPower(.6);
                    rightFront.setPower(-.6);
                    rightRear.setPower(-.6);
                }

                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);
                sleep(500);

                encReset();
*/
        leftFront.setPower(-.3);
        leftRear.setPower(-.3);
        rightFront.setPower(-.3);
        rightRear.setPower(-.3);
        while (Math.abs(leftFront.getCurrentPosition()) < 700 && opModeIsActive()) {
        telemetry.addData("Encoder Count", leftFront.getCurrentPosition());
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if (blockDetected) {
        GlyphWheels.setPower(0);
        }
        }
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        //GlyphWheels.setPower(0);
        encReset();

        //GlyphWheels.setPower(.6);

        leftFront.setPower(.3);
        leftRear.setPower(.3);
        rightFront.setPower(-.3);
        rightRear.setPower(-.3);
        while (Math.abs(leftFront.getCurrentPosition()) < 500 && opModeIsActive()) {
        telemetry.addData("Encoder Count", leftFront.getCurrentPosition());
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if (blockDetected) {
        GlyphWheels.setPower(0);
        }
        }
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        //GlyphWheels.setPower(0);
        encReset();


        //GlyphWheels.setPower(.6);
        leftFront.setPower(.3);
        leftRear.setPower(.3);
        rightFront.setPower(.3);
        rightRear.setPower(.3);
        while (Math.abs(leftFront.getCurrentPosition()) < 1400 && opModeIsActive()) {
        telemetry.addData("Encoder Count", leftFront.getCurrentPosition());
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if (blockDetected) {
        GlyphWheels.setPower(0);
        }
        }
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        //GlyphWheels.setPower(0);
        encReset();


        //GlyphWheels.setPower(.6);

        leftFront.setPower(-.3);
        leftRear.setPower(-.3);
        rightFront.setPower(.3);
        rightRear.setPower(.3);
        while (Math.abs(leftFront.getCurrentPosition()) < 400 && opModeIsActive()) {
        telemetry.addData("Encoder Count", leftFront.getCurrentPosition());
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if (blockDetected) {
        GlyphWheels.setPower(0);
        }
        }
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        GlyphWheels.setPower(0);
        encReset();

              /*  GlyphWheels.setPower(.6);
                leftFront.setPower(-.3);
                leftRear.setPower(-.3);
                rightFront.setPower(.3);
                rightRear.setPower(.3);
                while (Math.abs(leftFront.getCurrentPosition()) < 300 && opModeIsActive()) {
                    telemetry.addData("Encoder Count", leftFront.getCurrentPosition());
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                }
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);
                GlyphWheels.setPower(0);
                encReset();

                GlyphWheels.setPower(.6);
                leftFront.setPower(.3);
                leftRear.setPower(.3);
                rightFront.setPower(.3);
                rightRear.setPower(.3);
                while (Math.abs(leftFront.getCurrentPosition()) < 1000 && opModeIsActive()) {
                    telemetry.addData("Encoder Count", leftFront.getCurrentPosition());
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                }
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);
                GlyphWheels.setPower(0);
                encReset();*/

        sleep(750);
        GlyphWheels.setPower(.7);
        encReset();
        leftFront.setPower(-.7); //.75
        leftRear.setPower(-.7);
        rightFront.setPower(-.7);
        rightRear.setPower(-.7);
        while (Math.abs(leftFront.getCurrentPosition()) < 2200 && opModeIsActive()) {
        telemetry.addData("Encoder Count", leftFront.getCurrentPosition());
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        GlyphWheels.setPower(0);
        encReset();

                /*GlyphWheels.setPower(.4);
                sleep(1000);;
                score.setPower(.3);
                sleep(500);
                score.setPower(.4);
                sleep(1500);
                score.setPower(-.25);
                sleep(1000);

                leftFront.setPower(.5); //.75
                leftRear.setPower(.5);
                rightFront.setPower(.48);
                rightRear.setPower(.48);
                while (Math.abs(leftFront.getCurrentPosition()) < 300 && opModeIsActive()) {
                    telemetry.addData("Encoder Count", leftFront.getCurrentPosition());
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                }
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);
                encReset();*/
        leftRear.setPower(-.25);
        leftFront.setPower(-.25);
        rightRear.setPower(-.25);
        rightFront.setPower(-.25);
        sleep(1500);
        leftRear.setPower(.25);
        leftFront.setPower(.25);
        rightRear.setPower(.25);
        rightFront.setPower(.25);
        sleep(500);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        sleep(500);




        }
        while (opModeIsActive()) {
        telemetry.addData("End of code", 0);
        telemetry.update();
        }
        }
        }
        }


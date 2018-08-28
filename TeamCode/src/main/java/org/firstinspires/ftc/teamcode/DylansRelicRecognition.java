package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
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


/**
 * This OpMode illustrates the basics of using the Vuforia engine to determine
 * the identity of Vuforia VuMarks encountered on the field. The code is structured as
 * a LinearOpMode. It shares much structure with {@link ConceptVuforiaNavigation}; we do not here
 * duplicate the core Vuforia documentation found there, but rather instead focus on the
 * differences between the use of Vuforia for navigation vs VuMark identification.
 *
 * @see ConceptVuforiaNavigation
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained in {@link ConceptVuforiaNavigation}.
 */

@Autonomous(name="Relic_Blue_1", group ="Autonomous")
@Disabled
public class DylansRelicRecognition extends LinearOpMode {

    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    VuforiaLocalizer vuforia;
    private ElapsedTime runtime = new ElapsedTime();
    private double timeout;
    private DcMotor leftmotor;
    private DcMotor rightmotor;
    private DcMotor GlyphWheels;
    private DcMotor GlyphSlide;
    private DcMotor lights;
    private ColorSensor Color;
    private DistanceSensor Range;
    private Servo arm;
    private Servo Horizontal_jewel;
    private Servo tss;
    private DigitalChannel Touch;

    private int jewel = 0;
    private boolean cryptobox;

    private BNO055IMU imu;

    double wallDistance;

    boolean afterjewel = true;
    boolean Searching;
    boolean vuforiaHasRan = false;

    Orientation angles;

    public void encReset() throws InterruptedException {
        leftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void jewelRunner() throws InterruptedException {
        arm.setPosition(0.19);


        Horizontal_jewel.setPosition(.56);
        sleep(1000);
        if (Color.blue() > Color.red()) {
            sleep(2000);
            Horizontal_jewel.setPosition(0); //Arm Motion
            sleep(1000);
            arm.setPosition(0.50);
            sleep(500);
            Horizontal_jewel.setPosition(0.30);
            sleep(1000);
            Horizontal_jewel.setPosition(.56);
            sleep(1000);
            arm.setPosition(.9);
        } else if (Color.red() > Color.blue()) {
            sleep(1000);
            Horizontal_jewel.setPosition(1); //Arm Motion
            sleep(1000);
            arm.setPosition(0.50);
            sleep(500);
            Horizontal_jewel.setPosition(0.70);
            sleep(1000);
            Horizontal_jewel.setPosition(.56);
            sleep(1000);
            arm.setPosition(.9);
        }
    }


    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });


        telemetry.addLine()
                .addData("first angle z", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("second angle y", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("third angle x", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });


    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }



    @Override
    public void runOpMode() throws InterruptedException {
        leftmotor = (hardwareMap.dcMotor.get("leftmotor"));
        rightmotor = (hardwareMap.dcMotor.get("rightmotor"));
        rightmotor.setDirection(DcMotor.Direction.REVERSE);
        GlyphWheels = (hardwareMap.dcMotor.get("lift"));
        GlyphSlide = (hardwareMap.dcMotor.get("slide"));
        lights = (hardwareMap.dcMotor.get("lights"));
        arm = (hardwareMap.servo.get("arm"));
        Horizontal_jewel = (hardwareMap.servo.get("hj"));
        Color = (hardwareMap.colorSensor.get("color"));
        tss = (hardwareMap.servo.get("tss"));
        Touch = hardwareMap.get(DigitalChannel.class, "touch");
        Touch.setMode(DigitalChannel.Mode.INPUT);

        Range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");


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

        telemetry.addData("version: ", "1.1.3 | Now with gyro edition!");
        telemetry.update();
        waitForStart();

        runtime.reset();

        relicTrackables.activate();

        //while (opModeIsActive()) {

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

            //10.16 cm
            while(jewel < 1) {
                jewelRunner();
                jewel++;
            }

            timeout = runtime.milliseconds();

            //this is where the vuforia checks are, it won't run any of it if 5 seconds pass from the timeout above.
            while(!vuforiaHasRan && opModeIsActive() && timeout + 5000 > runtime.milliseconds()) {
                vuMark = RelicRecoveryVuMark.from(relicTemplate);
                telemetry.addData("Timeout number: ", timeout + 5000);
                telemetry.addData("Current time: ", runtime.milliseconds());
                telemetry.update();

                if (vuMark.equals(RelicRecoveryVuMark.LEFT)) {
                    vuforiaHasRan = true;
                    telemetry.addData("Read LEFT ", 0);
                    telemetry.update();
                    while (!cryptobox && opModeIsActive()) {
                        tss.setPosition(.4);
                        while (Touch.getState() && opModeIsActive()) {
                            leftmotor.setPower(.26);
                            rightmotor.setPower(.3);
                        }
                        cryptobox = true;

                    }

                    leftmotor.setPower(0);
                    rightmotor.setPower(0);
                    tss.setPosition(0);

                    sleep(1000);

                    encReset();
                    leftmotor.setPower(-.22);
                    rightmotor.setPower(-.25);
                    while (Math.abs(leftmotor.getCurrentPosition()) < 300) {
                        telemetry.addData("Encoder Count", leftmotor.getCurrentPosition());
                        telemetry.update();
                    }

                    while (angles.firstAngle < 105 && opModeIsActive()) {


                        //we figured out that telemetry.update() MUST BE USED to update
                        //the value recieved from angles.firstAngle, otherwise the number
                        //will never move
                        //that is stupid, but its what needs to be done


                        telemetry.addData("current angle: ", angles.firstAngle);
                        telemetry.update();
                        leftmotor.setPower(-.3);
                        rightmotor.setPower(.32);
                    }

                    leftmotor.setPower(0);
                    rightmotor.setPower(0);
                    sleep(500);

                    while (angles.firstAngle > 105 && opModeIsActive()) {
                        telemetry.addData("current angle: ", angles.firstAngle);
                        telemetry.update();
                        leftmotor.setPower(.3);
                        rightmotor.setPower(-.32);
                    }

                    leftmotor.setPower(0);
                    rightmotor.setPower(0);
                    sleep(1000);

                    //commenting out the if becuase it may not need it with how close the turn is to begin with
                    //and due to the fact that the gyro zeros on run, so it will be off if the robot is off on start
                    //if (angles.firstAngle < 95 && angles.firstAngle > 85) {
                    leftmotor.setPower(.25);
                    rightmotor.setPower(.29);
                    sleep(1500);

                    GlyphWheels.setPower(-.6);
                    leftmotor.setPower(0);
                    rightmotor.setPower(0);
                    sleep(900);

                    leftmotor.setPower(-.25);
                    rightmotor.setPower(-.25);
                    sleep(500);

                    GlyphWheels.setPower(0);
                    sleep(250);

                    leftmotor.setPower(0);
                    rightmotor.setPower(.2);
                    sleep(1200);

                    leftmotor.setPower(0.27);
                    rightmotor.setPower(0.27);
                    sleep(1500);

                    leftmotor.setPower(-.25);
                    rightmotor.setPower(-.25);
                    GlyphWheels.setPower(-.3);
                    sleep(1750);

                    leftmotor.setPower(0);
                    rightmotor.setPower(0);
                    GlyphWheels.setPower(0);

                    //}

                }else if (vuMark.equals(RelicRecoveryVuMark.CENTER) && timeout + 5000 > runtime.milliseconds()) {
                    vuforiaHasRan = true;
                    telemetry.addData("Read CENTER ", 0);
                    telemetry.update();

                    while (!cryptobox && opModeIsActive() || timeout + 1000 > runtime.milliseconds()) {
                        tss.setPosition(.4);
                        while (Touch.getState() && opModeIsActive()) {
                            leftmotor.setPower(.21);
                            rightmotor.setPower(.26);
                            lights.setPower(.2);
                        }
                        cryptobox = true;
                    }

                    leftmotor.setPower(0);
                    rightmotor.setPower(0);
                    tss.setPosition(0);
                    encReset();
                    sleep(1000);


                    leftmotor.setPower(0.20);
                    rightmotor.setPower(0.20);
                    while (Math.abs(leftmotor.getCurrentPosition()) < 700) {
                        telemetry.addData("Encoder Count", leftmotor.getCurrentPosition());
                        telemetry.update();
                    }
                    leftmotor.setPower(0);
                    rightmotor.setPower(0);
                    encReset();
                    while (angles.firstAngle < 100 && opModeIsActive()) {

                        //we figured out that telemetry.update() MUST BE USED to update
                        //the value recieved from angles.firstAngle, otherwise the number
                        //will never move
                        //that is stupid, but its what needs to be done
                        telemetry.addData("current angle: ", angles.firstAngle);
                        telemetry.update();
                        leftmotor.setPower(-.275);
                        rightmotor.setPower(.275);
                    }

                    leftmotor.setPower(0);
                    rightmotor.setPower(0);
                    sleep(500);

                    while (angles.firstAngle > 100 && opModeIsActive()) {
                        telemetry.addData("current angle: ", angles.firstAngle);
                        telemetry.update();
                        leftmotor.setPower(.22);
                        rightmotor.setPower(-.24);
                    }

                    leftmotor.setPower(0);
                    rightmotor.setPower(0);
                    sleep(1000);

                    //commenting out the if becuase it may not need it with how close the turn is to begin with
                    //and due to the fact that the gyro zeros on run, so it will be off if the robot is off on start
                    //if (angles.firstAngle < 95 && angles.firstAngle > 85) {
                    leftmotor.setPower(.25);
                    rightmotor.setPower(.36);
                    sleep(1500);
                    GlyphWheels.setPower(-1);
                    leftmotor.setPower(0);
                    rightmotor.setPower(0);
                    sleep(1000);
                    leftmotor.setPower(-.25);
                    rightmotor.setPower(-.25);
                    sleep(500);
                    GlyphWheels.setPower(0);
                    sleep(250);
                    leftmotor.setPower(0);
                    rightmotor.setPower(.2);
                    sleep(1200);
                    leftmotor.setPower(0.27);
                    rightmotor.setPower(0.31);
                    sleep(2000);
                    leftmotor.setPower(-.25);
                    rightmotor.setPower(-.25);
                    GlyphWheels.setPower(-.25);
                    sleep(1000);
                    leftmotor.setPower(0);
                    rightmotor.setPower(0);
                    GlyphWheels.setPower(0);


                } else if (vuMark.equals(RelicRecoveryVuMark.RIGHT) && timeout + 5000 > runtime.milliseconds()) {
                    vuforiaHasRan = true;
                    telemetry.addData("Read RIGHT ", 0);
                    telemetry.update();

                    while (!cryptobox && opModeIsActive() || timeout + 1000 > runtime.milliseconds()) {
                        tss.setPosition(.4);
                        while (Touch.getState() && opModeIsActive()) {
                            leftmotor.setPower(.21);
                            rightmotor.setPower(.26);
                            lights.setPower(.2);
                        }
                        cryptobox = true;
                    }

                    leftmotor.setPower(0);
                    rightmotor.setPower(0);
                    tss.setPosition(0);
                    encReset();
                    sleep(1000);


                    leftmotor.setPower(0.20);
                    rightmotor.setPower(0.24);
                    while (Math.abs(leftmotor.getCurrentPosition()) < 1200) {
                        telemetry.addData("Encoder Count", leftmotor.getCurrentPosition());
                        telemetry.update();
                    }
                    leftmotor.setPower(0);
                    rightmotor.setPower(0);
                    encReset();
                    while (angles.firstAngle < 100 && opModeIsActive()) {

                        //we figured out that telemetry.update() MUST BE USED to update
                        //the value recieved from angles.firstAngle, otherwise the number
                        //will never move
                        //that is stupid, but its what needs to be done
                        telemetry.addData("current angle: ", angles.firstAngle);
                        telemetry.update();
                        leftmotor.setPower(-.275);
                        rightmotor.setPower(.275);
                    }

                    leftmotor.setPower(0);
                    rightmotor.setPower(0);
                    sleep(500);

                    while (angles.firstAngle > 100 && opModeIsActive()) {
                        telemetry.addData("current angle: ", angles.firstAngle);
                        telemetry.update();
                        leftmotor.setPower(.22);
                        rightmotor.setPower(-.24);
                    }

                    leftmotor.setPower(0);
                    rightmotor.setPower(0);
                    sleep(1000);

                    //commenting out the if becuase it may not need it with how close the turn is to begin with
                    //and due to the fact that the gyro zeros on run, so it will be off if the robot is off on start
                    //if (angles.firstAngle < 95 && angles.firstAngle > 85) {
                    leftmotor.setPower(.23);
                    rightmotor.setPower(.36);
                    sleep(1500);
                    GlyphWheels.setPower(-1);
                    leftmotor.setPower(0);
                    rightmotor.setPower(0);
                    sleep(1000);
                    leftmotor.setPower(-.25);
                    rightmotor.setPower(-.25);
                    sleep(500);
                    GlyphWheels.setPower(0);
                    sleep(250);
                    leftmotor.setPower(0);
                    rightmotor.setPower(.2);
                    sleep(1200);
                    leftmotor.setPower(0.27);
                    rightmotor.setPower(0.33);
                    sleep(2000);
                    leftmotor.setPower(-.25);
                    rightmotor.setPower(-.25);
                    GlyphWheels.setPower(-.25);
                    sleep(1000);
                    leftmotor.setPower(0);
                    rightmotor.setPower(0);
                    GlyphWheels.setPower(0);


                }


            }
        if (!vuforiaHasRan) {
            telemetry.addData("Nothing read, TIMEOUT ", 0);
            telemetry.update();
            while (!cryptobox && opModeIsActive()) {
                tss.setPosition(.4);
                while (Touch.getState() && opModeIsActive()) {
                    leftmotor.setPower(.21);
                    rightmotor.setPower(.26);
                }
                cryptobox = true;

            }

            leftmotor.setPower(0);
            rightmotor.setPower(0);
            tss.setPosition(0);

            sleep(1000);

                    /*leftmotor.setPower(0.20);
                    rightmotor.setPower(0.2);
                    while (Math.abs(leftmotor.getCurrentPosition()) < 100) {

                        telemetry.addData("Encoder Count", leftmotor.getCurrentPosition());
                        telemetry.update();
*/
            while (angles.firstAngle < 100 && opModeIsActive()) {

                //we figured out that telemetry.update() MUST BE USED to update
                //the value recieved from angles.firstAngle, otherwise the number
                //will never move
                //that is stupid, but its what needs to be done

                telemetry.addData("current angle: ", angles.firstAngle);
                telemetry.update();
                leftmotor.setPower(-.35);
                rightmotor.setPower(.38);
            }

            leftmotor.setPower(0);
            rightmotor.setPower(0);
            sleep(500);

            while (angles.firstAngle > 100 && opModeIsActive()) {
                telemetry.addData("current angle: ", angles.firstAngle);
                telemetry.update();
                leftmotor.setPower(.28);
                rightmotor.setPower(-.25);
            }

            leftmotor.setPower(0);
            rightmotor.setPower(0);
            sleep(1000);

            //commenting out the if becuase it may not need it with how close the turn is to begin with
            //and due to the fact that the gyro zeros on run, so it will be off if the robot is off on start
            //if (angles.firstAngle < 95 && angles.firstAngle > 85) {
            leftmotor.setPower(.3);
            rightmotor.setPower(.33);
            sleep(1350);
            GlyphWheels.setPower(-.3);
            leftmotor.setPower(0);
            rightmotor.setPower(0);
            sleep(1000);
            leftmotor.setPower(-.25);
            rightmotor.setPower(-.25);
            sleep(500);
            GlyphWheels.setPower(0);
            sleep(250);
            leftmotor.setPower(0);
            rightmotor.setPower(.2);
            sleep(1200);
            leftmotor.setPower(0.3);
            rightmotor.setPower(0.32);
            sleep(2000);
            leftmotor.setPower(-.25);
            rightmotor.setPower(-.25);
            GlyphWheels.setPower(-.25);
            sleep(1250);
            leftmotor.setPower(0);
            rightmotor.setPower(0);
            GlyphWheels.setPower(0);
        }



        //}
        while(opModeIsActive()) {
            //end of program, just sits until op is over
            telemetry.addData("End of Code", 0);
            telemetry.update();
        }
    }
}


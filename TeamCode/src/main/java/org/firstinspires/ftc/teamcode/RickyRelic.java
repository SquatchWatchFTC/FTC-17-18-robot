package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.Gamepad;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.util.ElapsedTime;

        import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by 8868 on 4/5/2018.
 */
@TeleOp (name = "Relic_choosePosition", group = "TeleOp")
public class RickyRelic extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private Servo grabber;
    private Servo lifter;
    private DcMotor runner;

    double liftServo = 0;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        grabber = (hardwareMap.servo.get("grabber"));
        lifter = (hardwareMap.servo.get("lifter"));
        runner = (hardwareMap.dcMotor.get("runner"));
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

        if (gamepad1.a) {
            liftServo = liftServo + 0.01;
        } else if (gamepad1.x) {
            liftServo = liftServo - 0.01;
        }
        /**if (gamepad1.a && gamepad1.left_bumper){
         liftServo = liftServo + 0.002;
         }**/
        lifter.setPosition(liftServo);

        if (gamepad1.b) {

            grabber.setPosition(1);
        } else {
            grabber.setPosition(0);
        }
        if (gamepad1.dpad_up) {

            runner.setPower(.75);
        } else {
            runner.setPower(0);
        }
        if (gamepad1.dpad_down) {
            runner.setPower(-.3);
        } else {
            runner.setPower(0);
        }
    }
}


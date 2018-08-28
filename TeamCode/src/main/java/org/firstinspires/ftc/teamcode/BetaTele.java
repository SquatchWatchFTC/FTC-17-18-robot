
package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.ColorSensor;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.util.ElapsedTime;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.ColorSensor;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Created by 8868 on 1/15/2018.
 */

  @TeleOp (name = "BetaTele", group = "TeleOp")

public class BetaTele extends OpMode{
        private ElapsedTime runtime = new ElapsedTime();
        private DcMotor leftmotor;
        private DcMotor rightmotor;



        @Override
        public void init() {
                leftmotor = (hardwareMap.dcMotor.get("leftmotor"));
                leftmotor.setDirection(DcMotorSimple.Direction.REVERSE);
                rightmotor = (hardwareMap.dcMotor.get("rightmotor"));
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

                leftmotor.setPower(gamepad1.right_stick_y);
                rightmotor.setPower(gamepad1.left_stick_y);
        }
        @Override
        public void stop() {
        }

}



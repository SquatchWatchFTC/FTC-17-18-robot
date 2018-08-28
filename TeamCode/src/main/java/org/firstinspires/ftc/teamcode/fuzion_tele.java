package org.firstinspires.ftc.teamcode;


        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.ColorSensor;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.util.ElapsedTime;
        import com.qualcomm.robotcore.hardware.Servo;
/**
 * Created by 8868 on 11/27/2017.
 */



@TeleOp (name = "fuzion_tele" , group = "TeleOp")

@Disabled
    public class fuzion_tele extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftmotor;
    private DcMotor rightmotor;
    private DcMotor arm;
    private Servo servol;
    private Servo servor;
    private DcMotor ramp;


@Override
public void init(){
    telemetry.addData("Status", "Initialized");

    leftmotor = (hardwareMap.dcMotor.get("leftmotor"));
    rightmotor = (hardwareMap.dcMotor.get("rightmotor"));
    rightmotor.setDirection(DcMotor.Direction.REVERSE);
    arm = (hardwareMap.dcMotor.get("arm"));
    servol = (hardwareMap.servo.get("servol"));
    servor = (hardwareMap.servo.get("servor"));
    ramp = (hardwareMap.dcMotor.get("ramp"));

}

    @Override
    public void init_loop() {
    }
    @Override
    public void start(){
    runtime.reset();
    }
    @Override
    public void loop(){

        float leftY = gamepad1.left_stick_y;
        float rightY = gamepad1.right_stick_y;




        if(leftY > 0 && rightY < 0 || leftY < 0 && rightY > 0){
            leftmotor.setPower(leftY * .75);
            rightmotor.setPower(rightY * .75);
        }
        else if (gamepad1.right_bumper){
            leftmotor.setPower(leftY / 2);
            rightmotor.setPower(rightY / 2);
        }
        else {
            leftmotor.setPower(leftY);
            rightmotor.setPower(rightY);
        }


        if (gamepad2.a){

            servor.setPosition(.4);
        }
        else {
            servor.setPosition(1);
        }
        if (gamepad2.dpad_up){
            arm.setPower(-.75);
        } else if (gamepad2.dpad_down){
            arm.setPower(.75);
        } else{
            arm.setPower(0);
        }

        if (gamepad1.b){
            ramp.setPower(.25);
        } else if (gamepad1.a){
            ramp.setPower(-1);
        } else {
            ramp.setPower(0);
        }
    }



    @Override
    public void stop(){
        
    }


}
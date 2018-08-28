/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/*
 * This is an example LinearOpMode that shows how to use Kauai Labs navX Micro Robotics Navigation
 * Sensor. It assumes that the sensor is configured with a name of "navx".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name = "NavxAxisTest", group = "Sensor")
@Disabled
public class NavxMicroAxisTest extends LinearOpMode {

    /** In this sample, for illustration purposes we use two interfaces on the one gyro object.
     * That's likely atypical: you'll probably use one or the other in any given situation,
     * depending on what you're trying to do. {@link IntegratingGyroscope} (and it's base interface,
     * {@link Gyroscope}) are common interfaces supported by possibly several different gyro
     * implementations. {@link NavxMicroNavigationSensor}, by contrast, provides functionality that
     * is unique to the navX Micro sensor.
     */

    private DcMotor leftRear;
    private DcMotor rightRear;
    private DcMotor leftFront;
    private DcMotor rightFront;

    NavxMicroNavigationSensor navxMicro;
    boolean collision = false;
    float[] collisionVals = new float[3];
    AngularVelocity velocity;
    int threadPasses = 0;
    int loopPasses = 0;

    public void setCollisionState(boolean newVal) {
        collision = newVal;
    }

    public void setCollisionVals(float val1, float val2, float val3) {
        collisionVals[0] = val1;
        collisionVals[1] = val2;
        collisionVals[2] = val3;
    }
    Thread axisUpdate = new Thread(new Runnable() {
        //ElapsedTime timeout = new ElapsedTime();
        boolean collisionState = false;
        double threshold = 25.0;
        float lastX = 0, lastY = 0, lastZ = 0;
        float currX = 0, currY = 0, currZ = 0;
        float jerkX = 0, jerkY = 0, jerkZ = 0;
        @Override
        public void run() {
            //timeout.reset();
            //timeout.startTime();
            //while (timeout.seconds() < 30) {
            while (opModeIsActive()) {
                velocity = navxMicro.getAngularVelocity(AngleUnit.DEGREES);
                currX = velocity.xRotationRate; currY = velocity.yRotationRate; currZ = velocity.zRotationRate;
                jerkX = currX - lastX; jerkY = currY - lastY; jerkZ = currZ - lastZ;
                lastX = currX; lastY = currY; lastZ = currZ;
                if (Math.abs(jerkX) > threshold) {
                    collisionState = true;
                } else {
                    collisionState = false;
                }
                setCollisionVals(currX, currY, currZ);
                setCollisionState(collisionState);
            }
        }
    });

    // A timer helps provide feedback while calibration is taking place
    ElapsedTime timer = new ElapsedTime();

    @Override public void runOpMode() throws InterruptedException {
        navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");

        leftRear = (hardwareMap.dcMotor.get("left2"));
        rightRear = (hardwareMap.dcMotor.get("right2"));
        leftFront = (hardwareMap.dcMotor.get("left1"));
        rightFront = (hardwareMap.dcMotor.get("right1"));

        leftRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);

        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.log().add("Gyro Calibrating. Do Not Move!");

        timer.reset();


        while (navxMicro.isCalibrating())  {
            telemetry.addData("Calibrating..", 0);
            telemetry.update();
            Thread.sleep(50);
        }
        telemetry.log().clear(); telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear(); telemetry.update();

        // Wait for the start button to be pressed
        waitForStart();
        axisUpdate.start();
        telemetry.log().clear();

        leftRear.setPower(0.6);
        rightRear.setPower(0.68);
        leftFront.setPower(0.6);
        rightFront.setPower(0.68);

        sleep(1000);

        double timeout = timer.milliseconds();
        while (opModeIsActive() && timeout + 10000 > timer.seconds() && !collision) {
            telemetry.addData("Collsion state: ", collision);
            telemetry.addData("Vals", collisionVals[0]);
            telemetry.addData("Vals", collisionVals[1]);
            telemetry.addData("Vals", collisionVals[2]);
            telemetry.update();
        }
        leftRear.setPower(0);
        rightRear.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);

        while (opModeIsActive()){
            telemetry.addData("Collsion state: ", collision);
            telemetry.addData("Vals", collisionVals[0]);
            telemetry.addData("Vals", collisionVals[1]);
            telemetry.addData("Vals", collisionVals[2]);
            telemetry.update();
        }

        idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
    }
}
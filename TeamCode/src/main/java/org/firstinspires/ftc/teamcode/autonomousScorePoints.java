/* Copyright (c) 2021 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Score Points", group = "Autonomous")
//@Disabled
public class autonomousScorePoints extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor slideDrive = null;
    private DcMotor armDrive1 = null;
    private DcMotor armDrive2 = null;
    private Servo clawDrive = null;
    private Servo clawtator = null;
    long pace = 1;
    double speed = 0.5 * pace;
    double slowSpeed = 0.2 * pace;


    int increment = 0;

    @Override
    public void runOpMode() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        slideDrive = hardwareMap.get(DcMotor.class, "slide_drive");
        armDrive1 = hardwareMap.get(DcMotor.class, "arm_drive_1");
        armDrive2 = hardwareMap.get(DcMotor.class, "arm_drive_2");
        clawDrive = hardwareMap.get(Servo.class, "claw");
        clawtator = hardwareMap.get(Servo.class, "claw_rotate");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        slideDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        armDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
        armDrive2.setDirection(DcMotorSimple.Direction.FORWARD);





        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slideDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        clawDrive.setPosition(1);

        armDrive1.setPower(speed);
        armDrive2.setPower(speed);


        armDrive1.setTargetPosition(-2750);
        armDrive2.setTargetPosition(-2750);

        armDrive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slideDrive.setPower(speed);
        slideDrive.setTargetPosition(-350);
        slideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sleep(1500 / pace);

        straightDrive(600);

        slowStraightDrive(1000);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideDrive.setTargetPosition(-850);
        slideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        clawtator.setPosition(0.5);

        sleep(800 / pace);

        clawDrive.setPosition(0);
        clawtator.setPosition(0.3);
        armDrive1.setTargetPosition(-1800);
        armDrive2.setTargetPosition(-1800);

        armDrive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        sleep(1000 / pace);

        backDrive(600);


        rightDrive(1600);

        rightRotate(50);

        straightDrive(1200);

        rightDrive(500);

        backDrive(1400);

        straightDrive(1400);

        rightDrive(500);
        sleep(300 / pace);
        backDrive(1300);
        sleep(300 / pace);
        straightDrive(1300);
        sleep(300 / pace);
        rightRotate(50);
        rightDrive(350);
        sleep(150 / pace);
        backDrive(1300);

        straightDrive(500);

        leftDrive(500);


        while (increment < 4) {

            leftRotate(1250);


            clawDrive.setPosition(0);
            clawtator.setPosition(0.4);
            armDrive1.setTargetPosition(-900);
            armDrive2.setTargetPosition(-900);
            armDrive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            slideDrive.setTargetPosition(-1100);

            slowStraightDrive(1000);

            clawDrive.setPosition(1);

            sleep(500 / pace);

            armDrive1.setTargetPosition(-1000);
            armDrive2.setTargetPosition(-1000);
            armDrive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            slideDrive.setTargetPosition(0);
            slideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            sleep(50 / pace);

            int rightMove = 1700 - 100 * increment;

            rightDrive(rightMove);

            armDrive1.setTargetPosition(-2700);
            armDrive2.setTargetPosition(-2700);

            armDrive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);




            rightRotate(1300);

            straightDrive(400);

            slowStraightDrive(1700);

            leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            slideDrive.setTargetPosition(-900);
            slideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            clawtator.setPosition(0.5);

            leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            sleep(800 / pace);

            clawDrive.setPosition(0);
            clawtator.setPosition(0.3);
            armDrive1.setTargetPosition(-2100);
            armDrive2.setTargetPosition(-2100);

            armDrive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            sleep(1000 / pace);

            slowBackDrive(300);

            backDrive(200);

            rightDrive(rightMove);

            increment += 1;

            sleep(100 / pace);
        }
        slideDrive.setTargetPosition(0);
        slideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(100 / pace);
        slowStraightDrive(300);

        sleep(100 / pace);
    }
    //functions
    void straightDrive(int distance) {


        leftFrontDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightBackDrive.setPower(speed);

        sleep(distance / pace);

        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
    void slowStraightDrive(int distance) {


        leftFrontDrive.setPower(slowSpeed);
        rightFrontDrive.setPower(slowSpeed);
        leftBackDrive.setPower(slowSpeed);
        rightBackDrive.setPower(slowSpeed);

        sleep(distance / pace);

        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
    void backDrive(int distance) {


        leftFrontDrive.setPower(-speed);
        rightFrontDrive.setPower(-speed);
        leftBackDrive.setPower(-speed);
        rightBackDrive.setPower(-speed);

        sleep(distance / pace);

        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
    void slowBackDrive(int distance) {


        leftFrontDrive.setPower(-slowSpeed);
        rightFrontDrive.setPower(-slowSpeed);
        leftBackDrive.setPower(-slowSpeed);
        rightBackDrive.setPower(-slowSpeed);

        sleep(distance / pace);

        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
    void rightDrive(int distance) {


        leftFrontDrive.setPower(speed);
        rightFrontDrive.setPower(-speed);
        leftBackDrive.setPower(-speed);
        rightBackDrive.setPower(speed);

        sleep(distance / pace);

        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
    void leftDrive(int distance) {


        leftFrontDrive.setPower(-speed);
        rightFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightBackDrive.setPower(-speed);

        sleep(distance / pace);

        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
    void rightRotate(int distance) {

        leftFrontDrive.setPower(speed);
        rightFrontDrive.setPower(-speed);
        leftBackDrive.setPower(speed);
        rightBackDrive.setPower(-speed);

        sleep(distance / pace);

        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
    void leftRotate(int distance) {

        leftFrontDrive.setPower(-speed);
        rightFrontDrive.setPower(speed);
        leftBackDrive.setPower(-speed);
        rightBackDrive.setPower(speed);

        sleep(distance / pace);

        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

}

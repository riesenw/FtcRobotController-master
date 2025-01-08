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

import static org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive.PARAMS;

import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Omni With Claw v2", group = "Linear OpMode")
public class OmniWithExtendingArmClawv2 extends LinearOpMode {

    // Declare OpMode members for each of the  motors.
    private ElapsedTime runtime = new ElapsedTime();
    private Motor leftFrontDrive = null;
    private Motor leftBackDrive = null;
    private Motor rightFrontDrive = null;
    private Motor rightBackDrive = null;
    private DcMotor slideDrive = null;
    private DcMotor armDrive1 = null;
    private DcMotor armDrive2 = null;

    private Servo clawDrive = null;
    private Servo clawtator = null;
    private LazyImu lazyImu;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = new Motor(hardwareMap, "left_front_drive");
        leftBackDrive = new Motor(hardwareMap, "left_back_drive");
        rightFrontDrive = new Motor(hardwareMap, "right_front_drive");
        rightBackDrive = new Motor(hardwareMap, "right_back_drive");

        leftFrontDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        MecanumDrive drive = new MecanumDrive(leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive);

        slideDrive = hardwareMap.get(DcMotor.class, "slide_drive");
        armDrive1 = hardwareMap.get(DcMotor.class, "arm_drive_1");
        armDrive2 = hardwareMap.get(DcMotor.class, "arm_drive_2");

        clawDrive = hardwareMap.get(Servo.class, "claw");
        clawtator = hardwareMap.get(Servo.class, "claw_rotate");
        lazyImu = new LazyImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                PARAMS.logoFacingDirection, PARAMS.usbFacingDirection));

        //leftFrontDrive.setInverted(true);
        //leftBackDrive.setInverted(true);

        slideDrive.setDirection(DcMotor.Direction.FORWARD);
        slideDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armDrive1.setDirection(DcMotor.Direction.REVERSE);
        armDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armDrive2.setDirection(DcMotorSimple.Direction.FORWARD);
        armDrive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawDrive.setDirection(Servo.Direction.FORWARD);
        clawtator.setDirection(Servo.Direction.FORWARD);

        armDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //claw spiny thing code defining
        double servoTargetPosition = 0.65;
        double increment = 0.05;

        //open claw when init
        //clawDrive.setPosition(0.55);

        int savedSlidePosition = 50000;

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            //                                                                                         speed multiplier based on triggers
            double speedFactor1 = 1;

            if (gamepad1.right_trigger > 0) {
                speedFactor1 = 2;
            }
            if (gamepad1.left_trigger > 0) {
                speedFactor1 = 0.5;
            }

            double speedFactor2 = 1;

            if (gamepad1.right_trigger > 0) {
                speedFactor2 = 2;
            }
            if (gamepad1.left_trigger > 0) {
                speedFactor2 = 0.5;
            }

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -0.3 * gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = 0.3 * gamepad1.left_stick_x;
            double yaw = 0.3 * gamepad1.right_stick_x;

            double extend = 0.8 * gamepad2.right_stick_y;
            double moveArm = 1 * gamepad2.left_stick_y;

            //unused for now

            if (gamepad2.dpad_down) {
                savedSlidePosition = slideDrive.getCurrentPosition();
            }

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = (axial + lateral + yaw) * speedFactor1;
            double rightFrontPower = (axial - lateral - yaw) * speedFactor1;
            double leftBackPower = (axial - lateral + yaw) * speedFactor1;
            double rightBackPower = (axial + lateral - yaw) * speedFactor1;
            double slidePower = extend;
            double armPower = moveArm;


            double extensionDistance = (slideDrive.getCurrentPosition());
            double armPosition = (armDrive1.getCurrentPosition());

            // when extending the slide (positive extend, do not exceed maximum.
            // when retracting the slide (negative extend, do not go below minimum.

            //double closedSlide       = savedSlidePosition;
            //double openedSlide     =;

            if ((slideDrive.getCurrentPosition() >= 0) && (extend > 0)) slidePower = 0.0;
            if ((slideDrive.getCurrentPosition() <= -2200) && (extend < 0)) slidePower = 0.0;

            //repeat for arm

            if ((armDrive1.getCurrentPosition() >= 0) && (moveArm > 0)) armPower = 0.0;
            if ((armDrive1.getCurrentPosition() <= -4900) && (moveArm < 0)) armPower = 0.0;

            //claw moving stuff
            if (gamepad2.right_bumper) clawDrive.setPosition(0.5);
            if (!gamepad2.right_bumper) clawDrive.setPosition(0.7);


            //claw spiny stuff


            if ((gamepad2.left_bumper) && (servoTargetPosition < 0.65)) {
                servoTargetPosition += increment;
            }
            if ((gamepad2.left_trigger > 0.1) && (servoTargetPosition > 0)) {
                servoTargetPosition -= increment;
            }

            clawtator.setPosition(servoTargetPosition);


            double clawPosition = clawDrive.getPosition();


            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            drive.driveFieldCentric(
                    -0.6 * speedFactor1 * gamepad1.left_stick_x,
                    0.6 * speedFactor1 * gamepad1.left_stick_y,
                    -0.6 * speedFactor1 * gamepad1.right_stick_x,
                    lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES),
                    true
                    );


            slideDrive.setPower(slidePower);
            armDrive1.setPower(armPower);
            armDrive2.setPower(armPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Slide", "%4.2f", slidePower);
            telemetry.addData("Arm", "%4.2f", armPower);
            telemetry.addData("Speed Factor 1", "%4.2f", speedFactor1);
            telemetry.addData("Speed Factor 2", "%4.2f", speedFactor2);
            telemetry.addData("Slide Distance", "%4.2f", extensionDistance);
            telemetry.addData("arm position", "%4.2f", armPosition);
            telemetry.addData("Saved Slide Position", savedSlidePosition);
            telemetry.addData("claw position", "%4.2f", clawPosition);
            telemetry.addData("claw", gamepad2.right_trigger);
            telemetry.addLine("good");
            telemetry.addData("set claw position", "%4.2f", servoTargetPosition);
            telemetry.update();

        }
    }
}




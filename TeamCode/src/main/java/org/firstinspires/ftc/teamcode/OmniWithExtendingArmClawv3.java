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

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Omni With Claw v3", group = "Linear OpMode")
public class OmniWithExtendingArmClawv3 extends LinearOpMode {

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

    private List<Action> runningActions = new ArrayList<>();

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
        //initialize controllers
        GamepadEx driver1 = new GamepadEx(gamepad1);

        Mechanisms.ServoMacros servoMacros = new Mechanisms.ServoMacros(hardwareMap);
        Mechanisms.Extender extender = new Mechanisms.Extender(hardwareMap);
        Mechanisms.Pivot pivot = new Mechanisms.Pivot(hardwareMap);
        Mechanisms.Arm wrist = new Mechanisms.Arm(hardwareMap);
        Mechanisms.Claw claw = new Mechanisms.Claw(hardwareMap);

        double grabPosition = 0;
        double speedShift = 0.6;
        double scoreUp = 0;
        double specGrab = 0;

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        runningActions.add(new ParallelAction(
                extender.updateExtender(),
                pivot.updatePivot()
        ));

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while(opModeIsActive() && !isStopRequested()) {

            TelemetryPacket packet = new TelemetryPacket();

            // updated based on gamepads

            // update running actions
            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;


            //read controller buttons
            driver1.readButtons();

            if((driver1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER) && (scoreUp == 0) && (specGrab == 0))) {
                if (grabPosition == 0) {
                    runningActions.add(new SequentialAction(
                            servoMacros.closeGrabPosition(),
                            extender.closeGrabPosition(),
                            pivot.closeGrabPosition()
                    ));
                }
                if (grabPosition == 1) {
                    runningActions.add(new SequentialAction(
                            servoMacros.middleGrabPosition(),
                            extender.middleGrabPosition(),
                            pivot.middleGrabPosition()
                    ));
                }
                if (grabPosition == 2) {
                    runningActions.add(new SequentialAction(
                            servoMacros.farGrabPosition(),
                            extender.farGrabPosition(),
                            pivot.farGrabPosition()

                    ));
                }
                if (grabPosition == 3) {
                    runningActions.add(new SequentialAction(
                            servoMacros.pullIn(),
                            extender.pullIn(),
                            pivot.pullIn()

                    ));
                }
                if (grabPosition < 4) {
                    grabPosition += 1;
                }
                if (grabPosition == 4) {
                    grabPosition = 0;
                }
            }
            if((driver1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN) && (scoreUp == 0) && (specGrab == 0))) {
                if (grabPosition == 2) {
                    runningActions.add(new SequentialAction(
                            servoMacros.middleGrab(),
                            extender.middleGrab(),
                            pivot.middleGrab()

                    ));
                }
                if (grabPosition == 1) {
                    runningActions.add(new SequentialAction(
                            servoMacros.closeGrab(),
                            extender.closeGrab(),
                            pivot.closeGrab()

                    ));
                }
                if (grabPosition == 3) {
                    runningActions.add(new SequentialAction(
                            servoMacros.farGrab(),
                            extender.farGrab(),
                            pivot.farGrab()

                    ));
                }
                grabPosition = 0;
            }
            if ((driver1.wasJustReleased(GamepadKeys.Button.DPAD_DOWN)) && (scoreUp == 0) && (specGrab == 0)) {
                runningActions.add(new SequentialAction(
                        servoMacros.pullIn(),
                        extender.pullIn(),
                        pivot.pullIn()

                ));
            }
            if (grabPosition == 0) {
                speedShift = 0.8;
            }
            if (!(grabPosition == 0)  ) {
                speedShift = 0.5;
            }
            if (!(scoreUp == 0)) {
                speedShift = 0.4;
            }
            if (!(specGrab == 0)) {
                speedShift = 0.4;
            }
            if ((grabPosition == 0) && (driver1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) && (specGrab == 0)) {
                runningActions.add(new SequentialAction(
                        servoMacros.sampleUp(),
                        extender.sampleUp(),
                        pivot.sampleUp()
                ));
                scoreUp = 1;
            }
            if ((grabPosition == 0) && (scoreUp == 1) && (driver1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) && (specGrab == 0)) {
                runningActions.add(new SequentialAction(
                        servoMacros.sampleScore(),
                        extender.extendOut()

                ));
            }
            if ((grabPosition == 0) && (scoreUp == 1) && (driver1.wasJustReleased(GamepadKeys.Button.DPAD_UP)) && (specGrab == 0)) {
                runningActions.add(new SequentialAction(
                        servoMacros.sampleReturn(),
                        pivot.sampleReturn(),
                        extender.sampleReturn()
                ));
                scoreUp = 0;
            }
            if ((grabPosition == 0) && (scoreUp == 0) && (driver1.wasJustPressed(GamepadKeys.Button.A)) && (specGrab == 0)) {
                runningActions.add(new SequentialAction(
                        pivot.pivotSpecGrab(),
                        extender.extendIn(),
                        wrist.armSpec(),
                        claw.openClaw()
                ));
                specGrab += 1;
            }
            if ((grabPosition == 0) && (scoreUp == 0) && (driver1.wasJustReleased(GamepadKeys.Button.A)) && (specGrab == 1)) {
                runningActions.add(new SequentialAction(
                        claw.closeClaw()


                ));
                specGrab = 0;
            }
            if ((grabPosition == 0) && (scoreUp == 0) && (driver1.wasJustPressed(GamepadKeys.Button.B)) && (specGrab == 0)) {
                runningActions.add(new SequentialAction(
                        pivot.pivotClippingPos(),
                        extender.extendIn(),
                        wrist.armUp()
                ));
            }
            if ((grabPosition == 0) && (scoreUp == 0) && (driver1.wasJustPressed(GamepadKeys.Button.X)) && (specGrab == 0)) {
                runningActions.add(new SequentialAction(
                        extender.extendSpec()
                ));
            }
            if ((grabPosition == 0) && (scoreUp == 0) && (driver1.wasJustReleased(GamepadKeys.Button.X)) && (specGrab == 0)) {
                runningActions.add(new SequentialAction(
                        claw.openClaw(),
                        extender.extendIn(),
                        pivot.pivotClipDown()
                ));
            }




            drive.driveFieldCentric(
                    -speedShift * driver1.getLeftX(),
                    -speedShift * driver1.getLeftY(),
                    -speedShift * driver1.getRightX(),
                    lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES),
                    true
            );


            // Show the elapsed game time and wheel power.


        }
    }
}





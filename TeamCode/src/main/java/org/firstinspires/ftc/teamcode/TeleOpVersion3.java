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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp3", group = "TeleOp")
public class TeleOpVersion3 extends LinearOpMode {
    RobotHardware robot = new RobotHardware(this);
    double turboBoostFactor = 1.0;
    double joystickSensitivity = 0.25;

    @Override
    public void runOpMode() {

        robot.init();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.right_bumper) {
                turboBoostFactor = 2.0;
                gamepad1.rumble(100);
            } else {
                turboBoostFactor = 1.0;
            }

//            if (gamepad1.left_bumper) {
//                turboBoostFactor = 4.0;
//                gamepad1.rumble(100);
//            } else {
//                turboBoostFactor = 1.0;
//            }

            if (gamepad2.circle) {
                robot.setSweeperOn(true);
            }
            if (gamepad2.cross) {
                robot.setSweeperOn(false);
            }
            if (gamepad2.square) {
                robot.setUnjamSweeperOn(true);
            }

            if(gamepad2.triangle){
                robot.deployPixel();
            }

            if (gamepad2.dpad_left) {
                robot.moveArmToPickupPosition();
            }
            if (gamepad2.dpad_up) {
                robot.moveArmToCarryPosition();
            }
            if (gamepad2.dpad_right) {
                robot.moveArmToFlipPosition();
            }

//            if (gamepad1.dpad_up) {
//                robot.forward()
            }
//
//            if (gamepad1.dpad_left) {
//                robot.strafeLeft();
//            }
//
//            if (gamepad1.dpad_right) {
//                robot.strafeRight();
//            }
//
//            if (gamepad1.dpad_down) {
//                robot.backUp();
//            }

//            double axial = -gamepad1.left_stick_y * joystickSensitivity * turboBoostFactor;
//            double lateral = -gamepad1.left_stick_x * joystickSensitivity * turboBoostFactor;
//            double yaw = -gamepad1.right_stick_x * joystickSensitivity * turboBoostFactor;
//            robot.moveRobot(axial, lateral, yaw);

       //     double Arm = gamepad2.left_stick_y;
//            double Intake = gamepad2.right_stick_x;

           // robot.setSweeperPower(gamepad2.right_stick_x);
          //  robot.setArmPower(gamepad2.left_stick_y);

            if(gamepad1.options && gamepad2.options){
                robot.lauchAirPlane();
            }



        }
    }


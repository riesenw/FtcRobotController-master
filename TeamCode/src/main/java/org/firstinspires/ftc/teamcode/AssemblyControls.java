package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "AssemblyControls", group = "TeleOp")
public class AssemblyControls extends LinearOpMode {
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

            if (gamepad1.options) {
                turboBoostFactor = 2.0;
                gamepad1.rumble(100);
            } else {
                turboBoostFactor = 1.0;
            }

                if (gamepad1.circle) {
                    robot.setUnjamSweeperOn(false);
                    sleep(500);
                    robot.setSweeperOn(true);
                }
                if (gamepad1.square) {
                    robot.setSweeperOn(false);
                    sleep(500);
                    robot.setUnjamSweeperOn(true);
                }

        //    robot.setSweeperPower(gamepad2.left_stick_x);


            if (gamepad1.cross) {
                robot.setSweeperOn(false);
                robot.setUnjamSweeperOn(false);
                //   Stop
            }


            if (gamepad1.dpad_left) {
                robot.moveArmToPickupPosition();
            }
            if (gamepad1.dpad_up) {
                robot.moveArmToCarryPosition();
            }
            if (gamepad1.dpad_right) {
                robot.moveArmToFlipPosition();
            }

            double axial = -gamepad1.left_stick_y * joystickSensitivity * turboBoostFactor;
            double lateral = -gamepad1.left_stick_x * joystickSensitivity * turboBoostFactor;
            double yaw = -gamepad1.right_stick_x * joystickSensitivity * turboBoostFactor;
            robot.moveRobot(axial, lateral, yaw);

            //     double Arm = gamepad2.left_stick_y;
            double Intake = gamepad1.left_trigger;

            //  robot.setSweeperPower(gamepad2.right_stick_x);
            //  robot.setArmPower(gamepad2.left_stick_y);

  //          if(gamepad2.triangle){
    //            robot.deployPixel();
            }

            if(gamepad1.right_bumper && gamepad1.left_bumper){
                robot.lauchAirPlane();
            }

        }
    }


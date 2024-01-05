package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class AutonomousMaster {
    public static void go(LinearOpMode myOpMode, boolean isRed) {

        // set constants specific to red or blue team assignment: blue values are the default

        double TOWARD_AUDIENCE = 90;
        double AWAY_FROM_AUDIENCE = -90;
        double TOWARD_DRIVERS = 0;
        double AWAY_FROM_DRIVERS = 180;
        double ANGLE_TOWARD_BACK_CENTER = -75;

        double RIGHT_TARGET_ID = 1;
        double CENTER_TARGET_ID = 2;
        double LEFT_TARGET_ID = 3;


        // If robot is on the red team, change to the red team values
        if (isRed) {
            TOWARD_AUDIENCE = 90;
            AWAY_FROM_AUDIENCE = -90;
            TOWARD_DRIVERS = 180;
            AWAY_FROM_DRIVERS = 0;
            ANGLE_TOWARD_BACK_CENTER = 0;

            RIGHT_TARGET_ID = 4;
            CENTER_TARGET_ID = 5;
            LEFT_TARGET_ID = 6;

        }
        RobotHardware robot = new RobotHardware(myOpMode);
        robot.init();

        myOpMode.waitForStart();

        robot.strafe(AWAY_FROM_AUDIENCE, 6, 0.1);
        boolean objectDetected =
                robot.forward(AWAY_FROM_DRIVERS, 28, 0.1);
        if (objectDetected) {
            //We now know randomization was to the RIGHT position.
            robot.deployPixel();
            robot.backUp(TOWARD_DRIVERS, 11, 0.1);
            myOpMode.sleep(500);
            robot.rotateToHeading(AWAY_FROM_AUDIENCE);
//            robot.rotateToHeading(ANGLE_TOWARD_BACK_CENTER);
            myOpMode.sleep(500);

            robot.forward(TOWARD_AUDIENCE,14,0.1);
            myOpMode.sleep(500);
            robot.strafe(AWAY_FROM_DRIVERS,8, 0.1);
            myOpMode.sleep(3000);


            boolean targetReached = robot.autoDriveToTarget(4);
            // strafe two inches to the right to correct for camera not centered on robot


            myOpMode.sleep(3000);


            robot.strafeRight(robot.getHeading()+ 90, 2.0, 0.1);
            myOpMode.sleep(1000);
            robot.moveArmToFlipPosition();
            myOpMode.telemetry.addData("Target Reached", targetReached);
            myOpMode.telemetry.update();
            myOpMode.sleep(3000);
            robot.moveArmToCarryPosition();
            robot.strafe(AWAY_FROM_DRIVERS, 16, 0.2);


        } else {
            robot.backUp(TOWARD_DRIVERS, 14, 0.1);
            myOpMode.sleep(500);
            robot.strafeLeft(TOWARD_AUDIENCE, 8, 0.1);
            myOpMode.sleep(500);
            objectDetected =
                    robot.forward(AWAY_FROM_DRIVERS, 22, 0.1);
            myOpMode.sleep(500);
            if (objectDetected) {
                // We now know that the randomization was to center
                robot.deployPixel();
                robot.backUp(TOWARD_DRIVERS, 2, 0.1);

            } else {
                robot.backUp(TOWARD_DRIVERS, 6.5, 0.1);
                myOpMode.sleep(500);
                robot.strafeLeft(TOWARD_AUDIENCE, 8, 0.1);
                robot.deployPixel();
                robot.backUp(TOWARD_DRIVERS, 1.5, 0.1);
            }
        }
        myOpMode.sleep(30000);

    }
}



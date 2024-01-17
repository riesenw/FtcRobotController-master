package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Autonomous Backboard Blue Championship", group = "Autonomous")


public class BlueBackboardChampionship  extends LinearOpMode {

        public enum Direction {
            TOWARD_AUDIENCE(90),
            AWAY_FROM_AUDIENCE(270),
            TOWARD_DRIVERS(180),
            AWAY_FROM_DRIVERS(0);

            public final double heading;

            Direction(double heading) {this.heading = heading;}
        }

        public RobotHardware robot = new RobotHardware (this);

        @Override
        public void runOpMode() throws InterruptedException {
            robot.init();
            waitForStart();

            robot.strafeRight(Direction.TOWARD_AUDIENCE.heading, 6, 0.1);
            boolean objectDetected =
                    robot.forward(Direction.AWAY_FROM_DRIVERS.heading, 28, 0.1);
            sleep(500);
            if (objectDetected) {
                robot.deployPixel();
                robot.backUp(Direction.TOWARD_DRIVERS.heading, 8, 0.1);
                sleep(500);
                robot.rotateToHeading(270);
                sleep(500);
                robot.forward(Direction.AWAY_FROM_AUDIENCE.heading, 10, 0.1);
//                robot.forward(Direction.AWAY_FROM_AUDIENCE);


//                robot.forward(Direction.AWAY_FROM_AUDIENCE.heading, 8, 0.1);
//                robot.moveArmToFlipPosition();
//                robot.moveArmToCarryPosition();
//                robot.strafeRight(Direction.AWAY_FROM_DRIVERS.heading, 12, 0.1);
                sleep(5000);
            }
        }
    }





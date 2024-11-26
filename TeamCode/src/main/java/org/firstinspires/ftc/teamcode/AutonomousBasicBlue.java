/* Copyright (c) 2022 FIRST. All rights reserved.
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


@Autonomous(name = "Autonomous Basic Blue", group = "Autonomous")

public class AutonomousBasicBlue extends LinearOpMode {

// There are only a few difference between the red side autonomous and the
// blue side autonomous.  This opMode calls on the operations common to
// both and passes the information of which team the robot is on.


    @Override
    public void runOpMode() throws InterruptedException {


        // set constants specific to red or blue team assignment: blue values are the default


        double TOWARD_AUDIENCE = 90;
        double AWAY_FROM_AUDIENCE = -90;
        double TOWARD_DRIVERS = 0;
        double AWAY_FROM_DRIVERS = 180;
        double ANGLE_TOWARD_BACK_CENTER = 0;


        RobotHardware robot = new RobotHardware(this);
        robot.init();

        waitForStart();
//        robot.strafe(90, 6, 0.2); // this goes away from audience
//        robot.strafe(-90, 6, 0.2); // this goes toward audience

//        sleep(60000);



        robot.strafe(TOWARD_AUDIENCE, 6, 0.2);

        //first pixle drop location

        boolean objectDetected =
                robot.forward(TOWARD_DRIVERS, 28, 0.25);
        if (objectDetected) {
            //We now know randomization was to the RIGHT position.
            robot.deployPixel();
//            robot.backUp(TOWARD_DRIVERS, 9, 0.2);
            robot.backUp(AWAY_FROM_DRIVERS,4,0.2);

//            sleep(500);
//            robot.rotateToHeading(AWAY_FROM_AUDIENCE);
//            sleep(250);
//
//            robot.forward(AWAY_FROM_AUDIENCE, 27, 0.2);
//            sleep(1000);
////
//            robot.strafeRight(AWAY_FROM_DRIVERS, 1.5, 0.1);
//            robot.moveArmToFlipPosition();
////            myOpMode.telemetry.addData("Target Reached", targetReached);
////            myOpMode.telemetry.update();
//            sleep(3000);
//            robot.moveArmToCarryPosition();
//            sleep(1000);
//            robot.backUp(TOWARD_AUDIENCE, 1, 0.4);
//            robot.strafeRight(TOWARD_DRIVERS, 14.5, 0.4);
//            robot.forward(AWAY_FROM_AUDIENCE, 5, 0.2);

            //second pixle drop location
            //......................................................................................

        } else {
//            go to pixle drop location 2
            robot.backUp(AWAY_FROM_DRIVERS, 3, 0.4);
//            sleep(500);
            robot.strafeRight(AWAY_FROM_AUDIENCE, 8.5, 0.4);
//            sleep(500);
            objectDetected =
                    robot.forward(TOWARD_DRIVERS, 11, 0.4);
            sleep(500);
            if (objectDetected) {
//                drop purple pixle
                robot.backUp(AWAY_FROM_DRIVERS,1.5,0.1);
                robot.deployPixel();

                robot.backUp(AWAY_FROM_DRIVERS,30,0.2);

//                robot.backUp(TOWARD_DRIVERS, 10, 0.4);
//                //go to backbord
//                robot.rotateToHeading(AWAY_FROM_AUDIENCE);
//                sleep(500);
//                robot.forward(AWAY_FROM_AUDIENCE, 35, 0.25);
//                sleep(125);
//                robot.strafeLeft(AWAY_FROM_DRIVERS,2,0.2);
//                sleep(500);
//                //score pixle
//                robot.moveArmToFlipPosition();
//                sleep(3000);
//                robot.moveArmToCarryPosition();
//                sleep(1000);
//                robot.backUp(TOWARD_AUDIENCE, 1, 0.4);
//                robot.strafeRight(TOWARD_DRIVERS, 18, 0.4);
//                robot.forward(AWAY_FROM_AUDIENCE, 5, 0.2);

                //..................................................................................
            } else {
//                go to pixle drop location 3
                robot.backUp(AWAY_FROM_DRIVERS, 10, 0.2);
                sleep(250);

                robot.rotateToHeading(AWAY_FROM_AUDIENCE);
                sleep(250);
                robot.forward(AWAY_FROM_AUDIENCE,16,0.4);
                robot.backUp(TOWARD_AUDIENCE,6,0.2);
                robot.deployPixel();
//                go to backbord
                robot.backUp(TOWARD_AUDIENCE, 17, 0.4);
//                robot.rotateToHeading(TOWARD_AUDIENCE);
//                sleep(250);
//
//                robot.strafeLeft(AWAY_FROM_DRIVERS,18,3);
//                robot.forward(AWAY_FROM_AUDIENCE,21,0.3);
//                robot.strafeLeft(AWAY_FROM_DRIVERS,4,0.4);
////                score pixle
//                sleep(250);
//                robot.moveArmToFlipPosition();
//                sleep(3000);
//                robot.moveArmToCarryPosition();
//                sleep(1000);
//                robot.backUp(TOWARD_AUDIENCE, 1, 0.4);
//                robot.strafeRight(TOWARD_DRIVERS, 19.5, 0.4);
//                robot.forward(AWAY_FROM_AUDIENCE, 4.5, 0.3);


            }
        }
        sleep(30000);

    }

}


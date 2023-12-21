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


@Autonomous(name = "Auto Component Test", group = "Autonomous")


public class AutonomousComponentTest extends LinearOpMode {

    public enum Direction {
        TOWARD_AUDIENCE(90),
        AWAY_FROM_AUDIENCE(-90),
        TOWARD_DRIVERS(180),
        AWAY_FROM_DRIVERS(0);

        public final double heading;

        Direction(double heading) {
            this.heading = heading;
        }
    }

    public RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init();
        waitForStart();
        robot.rotateToHeading(Direction.TOWARD_AUDIENCE.heading);
        sleep(1000);
        robot.rotateToHeading(Direction.TOWARD_DRIVERS.heading);
        sleep(1000);
        robot.rotateToHeading(Direction.AWAY_FROM_AUDIENCE.heading);
        sleep(1000);
        robot.rotateToHeading(Direction.AWAY_FROM_DRIVERS.heading);
        sleep(1000);


//        robot.strafeRight(Direction.AWAY_FROM_AUDIENCE.heading, 8,0.1);
//        boolean objectDetected =
//                robot.forward(Direction.AWAY_FROM_DRIVERS.heading, 28, 0.1);
//        sleep(500);
//        if (objectDetected) {
//            robot.deployPixel();
//        } else {
//            robot.backUp(Direction.TOWARD_DRIVERS.heading, 14, 0.1);
//            sleep(500);
//            robot.strafeLeft(Direction.TOWARD_AUDIENCE.heading, 8, 0.1);
//            sleep(500);
//            objectDetected =
//                    robot.forward(Direction.AWAY_FROM_DRIVERS.heading, 24, 0.1);
//            sleep(500);
//            if (objectDetected){
//                robot.deployPixel();
//            }
//        }
        sleep(30000);
    }
}

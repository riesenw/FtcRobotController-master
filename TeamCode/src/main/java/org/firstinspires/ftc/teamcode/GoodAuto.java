package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.ArrayList;
import java.util.List;

@Config
//Need the autnomous tag in order for it show up on driver station as an autonomous program
// You can also set the name of the autonomous and the group
@Autonomous(name = "GOOD_AUTONOMOUS", group = "Autonomous")
public class GoodAuto extends LinearOpMode {
    private List<Action> runningActions = new ArrayList<>();

    @Override
    public void runOpMode() {
        //set the starting position
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));

        //initialize our roadrunner drivetrain
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        //initialize claw and lift from our mechanisms file
        //Mechanisms.Claw claw = new Mechanisms.Claw(hardwareMap);
        //Mechanisms.Lift lift = new Mechanisms.Lift(hardwareMap);
        Mechanisms.Arm wrist = new Mechanisms.Arm(hardwareMap);
        Mechanisms.Claw claw = new Mechanisms.Claw(hardwareMap);
        Mechanisms.Extender extender = new Mechanisms.Extender(hardwareMap);
        Mechanisms.Pivot pivot = new Mechanisms.Pivot(hardwareMap);
        Mechanisms.MotorMacros motorMacros = new Mechanisms.MotorMacros(hardwareMap);
        Mechanisms.ServoMacros servoMacros = new Mechanisms.ServoMacros(hardwareMap);


        TrajectoryActionBuilder traj1 = drive.actionBuilder(initialPose)
                .afterTime(0, servoMacros.sampleUp())
                .afterTime(0, pivot.sampleUp())
                .afterTime(1, extender.sampleUp())
                .splineToConstantHeading(new Vector2d(8.5, 19), Math.toRadians(180))
                .turnTo(Math.toRadians(-25))
                .splineToConstantHeading(new Vector2d(3.5, 19), Math.toRadians(180))
                .afterTime(0, servoMacros.sampleScore())
                .afterTime(0.5, servoMacros.sampleReturn())
                .waitSeconds(0.5);



                //simple movement, spline to a linear heading, so it will go to position
                //(10, 10) with a heading of 90 degrees

                //move lift up after 1.5 seconds
                /*.afterTime(1.5, lift.liftUp())*/


        TrajectoryActionBuilder traj2 = traj1.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(11.75, 11.75), Math.toRadians(180))
                .afterTime(0, servoMacros.farGrabPosition())
                .afterTime(0, pivot.farGrab())
                .afterTime(0, extender.farGrab())
                .waitSeconds(1.5)
                .splineToConstantHeading(new Vector2d(11.75, 15.75), Math.toRadians(180))
                .afterTime(0, servoMacros.farGrab())
                .waitSeconds(1)
                .afterTime(0, servoMacros.sampleUp())
                .afterTime(0, pivot.sampleUp())
                .afterTime(1, extender.sampleUp())
                .splineToConstantHeading(new Vector2d(3.5, 21), Math.toRadians(180))
                .afterTime(1, servoMacros.sampleScore())
                .afterTime(1.5, servoMacros.sampleReturn());




        TrajectoryActionBuilder traj3 = traj2.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(11.25, 11.5), Math.toRadians(180))
                .turnTo(Math.toRadians(0))
                .afterTime(0, servoMacros.farGrabPosition())
                .afterTime(0, pivot.farGrab())
                .afterTime(0, extender.farGrab())
                .waitSeconds(1.5)
                .splineToConstantHeading(new Vector2d(11.25, 15.5), Math.toRadians(180))
                .afterTime(0, servoMacros.farGrab())
                .waitSeconds(1)
                .afterTime(0, servoMacros.sampleUp())
                .afterTime(0, pivot.sampleUp())
                .afterTime(1, extender.sampleUp())
                .splineToConstantHeading(new Vector2d(3.5, 21), Math.toRadians(180))
                .turnTo(Math.toRadians(-25))
                .afterTime(0, servoMacros.sampleScore())
                .afterTime(0.5, servoMacros.sampleReturn());



        TrajectoryActionBuilder traj4 = traj3.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(12.25, 17), Math.toRadians(180))
                .turnTo(Math.toRadians(15))
                .afterTime(0, servoMacros.farGrabPosition())
                .afterTime(0, pivot.farGrab())
                .afterTime(0, extender.farGrab())
                .waitSeconds(1.5)
                .splineToConstantHeading(new Vector2d(12.25, 21), Math.toRadians(180))
                .afterTime(0, servoMacros.farGrab())
                .waitSeconds(1)
                .afterTime(0, servoMacros.sampleUp())
                .afterTime(0, pivot.sampleUp())
                .afterTime(1, extender.sampleUp())
                .splineToConstantHeading(new Vector2d(3.5, 21), Math.toRadians(180))
                .turnTo(Math.toRadians(-25))
                .afterTime(0, servoMacros.sampleScore())
                .afterTime(0.5, servoMacros.sampleReturn())
                .afterTime(1.5, pivot.sampleReturn())
                .afterTime(1.7, extender.sampleReturn())
                .waitSeconds(0.5)
                .turnTo(Math.toRadians(0));






        // actions that need to happen on init; for instance, a claw tightening.
        //Actions.runBlocking(claw.closeClaw());

        // any actions in here will run in initialization
        Actions.runBlocking(new ParallelAction(
                claw.closeClaw()
        ));

        //wait for autonomous to start
        waitForStart();

        //check if stop is requested
        if (isStopRequested()) return;

        //run actions sequentially, so it will run each action in order
        runningActions.add(new SequentialAction(
                traj1.build(),
                traj2.build(),
                traj3.build(),
                traj4.build()


                ));


        //this is just to continually update our extender and pivot P loop
        runningActions.add(new ParallelAction(
                extender.updateExtender(),
                pivot.updatePivot()
        ));

        while (opModeIsActive() && !isStopRequested()) {
            TelemetryPacket packet = new TelemetryPacket();
            // update running actions
            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;
            sleep(5);

        }

    }
}
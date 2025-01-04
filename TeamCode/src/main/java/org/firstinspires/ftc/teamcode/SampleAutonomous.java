package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.ArrayList;
import java.util.List;

@Config
//Need the autnomous tag in order for it show up on driver station as an autonomous program
// You can also set the name of the autonomous and the group
@Autonomous(name = "SAMPLE_AUTONOMOUS", group = "Autonomous")
public class SampleAutonomous extends LinearOpMode {
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


        TrajectoryActionBuilder traj1 = drive.actionBuilder(initialPose);
                //simple movement, spline to a linear heading, so it will go to position
                //(10, 10) with a heading of 90 degrees

                //move lift up after 1.5 seconds
                /*.afterTime(1.5, lift.liftUp())*/


        TrajectoryActionBuilder traj2 = traj1.endTrajectory().fresh()
                //simple movement, spline to a linear heading, so it will go to it's original position
                //(0,0) with a heading of 0 degrees
                .splineToLinearHeading(new Pose2d(32, 0, Math.toRadians(0)), Math.toRadians(0))
                .afterTime(0, extender.extendSpec())
                .afterTime(0.5, claw.openClaw())
                .afterTime(0.5, extender.extendIn())
                .afterTime(0.5, pivot.pivotClipDown())
                .waitSeconds(0.5)
                //.splineToLinearHeading(new Pose2d(15, -40, Math.toRadians(0)), Math.toRadians(315))
                .splineToConstantHeading(new Vector2d(15, -38), Math.toRadians(270), new TranslationalVelConstraint(50))
                .splineToConstantHeading(new Vector2d(55, -40), Math.toRadians(300), new TranslationalVelConstraint(50))
                .splineToConstantHeading(new Vector2d(15, -56), Math.toRadians(300), new TranslationalVelConstraint(50))
                //.splineToConstantHeading(new Vector2d(55, -56), Math.toRadians(300), new TranslationalVelConstraint(50))
                //.splineToConstantHeading(new Vector2d(15, -78), Math.toRadians(300), new TranslationalVelConstraint(50))
                //.splineToConstantHeading(new Vector2d(50, -63), Math.toRadians(300), new TranslationalVelConstraint(30))
                //.splineToConstantHeading(new Vector2d(15, -78), Math.toRadians(300), new TranslationalVelConstraint(30))
                //move lift down after 5 inches traveled
                /*.afterDisp(5.0, lift.liftDown()0*/;

        TrajectoryActionBuilder traj3 = traj2.endTrajectory().fresh()
                //position for grab
                .splineToConstantHeading(new Vector2d(15, -50), Math.toRadians(90), new TranslationalVelConstraint(50))
                .turn(Math.toRadians(180))
                .afterTime(0, pivot.pivotSpecGrab())
                .afterTime(0, extender.extendGrab())
                .afterTime(0, wrist.armSpec())
                .afterTime(0, claw.openClaw())
                .splineToConstantHeading(new Vector2d(7, -50), Math.toRadians(0), new TranslationalVelConstraint(20))
                .afterTime(0, claw.closeClaw())
                .waitSeconds(0.2)
                .afterTime(0, pivot.pivotClippingPos2())
                .afterTime(0, extender.extendIn())
                .afterTime(0, wrist.armUp())
                //.waitSeconds(0.5)
                .turn(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(32, -6), Math.toRadians(0), new TranslationalVelConstraint(50))
                .afterTime(0, extender.extendSpec())
                .afterTime(0.5, claw.openClaw())
                .afterTime(0.5, extender.extendIn())
                .afterTime(0.5, pivot.pivotClipDown());

        TrajectoryActionBuilder traj4 = traj3.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(15, -50), Math.toRadians(90), new TranslationalVelConstraint(50))
                .turn(Math.toRadians(180))
                .afterTime(0, pivot.pivotSpecGrab())
                .afterTime(0, extender.extendGrab())
                .afterTime(0, wrist.armSpec())
                .afterTime(0, claw.openClaw())
                .splineToConstantHeading(new Vector2d(8, -50), Math.toRadians(0), new TranslationalVelConstraint(20))
                .afterTime(0, claw.closeClaw())
                .waitSeconds(0.2)
                .afterTime(0, pivot.pivotClippingPos3())
                .afterTime(0, extender.extendIn())
                .afterTime(0, wrist.armUp())
                //.waitSeconds(0.5)
                .turn(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(36, -12), Math.toRadians(0), new TranslationalVelConstraint(50))
                .afterTime(0, extender.extendSpec())
                .afterTime(0.5, claw.openClaw())
                .afterTime(0.5, extender.extendIn())
                .afterTime(0.5, pivot.pivotClipDown())
                .waitSeconds(0.5);





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
                        wrist.armUp(),
                        pivot.pivotClippingPos(),
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
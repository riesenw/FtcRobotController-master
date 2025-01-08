package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Mechanisms {
    //class to create a claw
    public static class Claw {
        private Servo claw;
        //create the claw object from hardware map

        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "claw");
        }

        //implement action class in our close claw function.

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //when closeclaw is run, set the claw to closed position
                claw.setPosition(0.7);
                return false;
            }
        }
        //allow the function to be able to called from other files
        public Action closeClaw() {
            return new Claw.CloseClaw();
        }
        //create an openclaw function by implementing action class

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //when openclaw is run, set the claw to the open position
                claw.setPosition(0.5);
                return false;
            }
        }
        //allow the function to be able to be called from other files
        public Action openClaw() {
            return new Claw.OpenClaw();
        }
    }

    public static class Arm {
        private Servo arm;
        //create the claw object from hardware map

        public Arm(HardwareMap hardwareMap) {
            arm = hardwareMap.get(Servo.class, "claw_rotate");
        }

        //implement action class in our close claw function.

        public class ArmUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //this is for arm in
                arm.setPosition(0.45);
                return false;
            }
        }
        //allow the function to be able to called from other files
        public Action armUp() {
            return new Arm.ArmUp();
        }
        //create an openclaw function by implementing action class

        public class ArmDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //this is lb and is for the submersible
                arm.setPosition(0);
                return false;
            }
        }
        //allow the function to be able to be called from other files
        public Action armDown() {
            return new Arm.ArmDown();
        }

        public class ArmSpec implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //when closeclaw is run, set the claw to closed position
                arm.setPosition(0.35);
                return false;
            }
        }
        //allow the function to be able to called from other files
        public Action armSpec() {
            return new Arm.ArmSpec();
        }

    }

    //lift class (this will require an encoder plugged into the motor)
    public static class Pivot {
        public Motor leftPivot;
        public Motor rightPivot;

        //create lift from hardwaremap and initialize it

        public Pivot(HardwareMap hardwareMap) {
            //initialize our lift from hardwareMap
            leftPivot = new Motor(hardwareMap, "arm_drive_1", Motor.GoBILDA.RPM_60);
            //set the lift motor direction
            leftPivot.setInverted(true);
            //set position coefficient of the lift, (p value)
            //initialize our lift from hardwareMap
            rightPivot = new Motor(hardwareMap, "arm_drive_2", Motor.GoBILDA.RPM_60);
            //set the lift motor direction
            rightPivot.setInverted(false);
            rightPivot.setRunMode(Motor.RunMode.PositionControl);
            rightPivot.setPositionCoefficient(0.0016);
            leftPivot.setPositionCoefficient(0.0016);
            leftPivot.setRunMode(Motor.RunMode.PositionControl);
        }
        public class resetPivot implements Action {

            // actions are formatted via telemetry packets as below

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                //set the target position of the lift to 3000 ticks
                //leftPivot.resetEncoder();
                leftPivot.resetEncoder();
                rightPivot.resetEncoder();
                return false;
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off2
            }
        }

        public Action resetPivot() {
            return new resetPivot();
        }
        public class PivotClippingPos implements Action {

            // actions are formatted via telemetry packets as below

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                //set the target position of the lift to 3000 ticks
                //leftPivot.setTargetPosition(1300);
                leftPivot.setTargetPosition(-2500);
                rightPivot.setTargetPosition(-2500);
                return false;
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off2
            }
        }

        public Action pivotClippingPos() {
            return new PivotClippingPos();
        }

        public class PivotClipDown implements Action {

            // actions are formatted via telemetry packets as below

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                //set the target position of the lift to 3000 ticks
                leftPivot.setTargetPosition(-2000);
                rightPivot.setTargetPosition(-2000);
                return false;
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off2

            }

        }
        public Action pivotClipDown() {
            return new PivotClipDown();
        }

        public class PivotSpecGrab implements Action {

            // actions are formatted via telemetry packets as below

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                //set the target position of the lift to 3000 ticks
                leftPivot.setTargetPosition(-850);
                rightPivot.setTargetPosition(-850);
                return false;
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off2
            }
        }
        public Action pivotSpecGrab() {
            return new PivotSpecGrab();
        }

        /*public class PivotSpecDown implements Action {

            // actions are formatted via telemetry packets as below

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                //set the target position of the lift to 3000 ticks
                pivot.setTargetPosition(1000);
                return false;
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off2
            }
        }
        public Action pivotSpecDown() {
            return new PivotSpecDown();
        }*/

        public class UpdatePivot implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                //set the target position of the lift to 3000 ticks
                if (!leftPivot.atTargetPosition()) {
                    leftPivot.set(0.9);
                }
                if (!rightPivot.atTargetPosition()) {
                    rightPivot.set(0.9);
                }
                return true;
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off2
            }
        }
        public Action updatePivot() {
            return new UpdatePivot();
        }
        public int pivotPosition()
        {
            return rightPivot.getCurrentPosition();
        }

    }

    public static class Extender {
        private Motor extender;
        //create the claw object from hardware map

        public Extender(HardwareMap hardwareMap) {
            extender = new Motor(hardwareMap, "slide_drive", Motor.GoBILDA.RPM_312);
            extender.setRunMode(Motor.RunMode.PositionControl);
            extender.setPositionCoefficient(0.0025);
        }

        //implement action class in our close claw function.
        public class resetExtender implements Action {

            // actions are formatted via telemetry packets as below

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                //set the target position of the lift to 3000 ticks
                //leftPivot.resetEncoder();
                extender.resetEncoder();
                return false;
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off2
            }
        }
        public Action resetExtender() {
            return new Extender.resetExtender();
        }


        public class ExtendIn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //when closeclaw is run, set the claw to closed position
                extender.setTargetPosition(0);
                return false;
            }
        }
        //allow the function to be able to called from other files
        public Action extendIn() {
            return new Extender.ExtendIn();
        }
        //create an openclaw function by implementing action class

        public class ExtendOut implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //when openclaw is run, set the claw to the open position
                extender.setTargetPosition(-2200);
                return false;
            }
        }
        //allow the function to be able to be called from other files
        public Action extendOut() {
            return new Extender.ExtendOut();
        }

        public class ExtendSpec implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //when openclaw is run, set the claw to the open position
                extender.setTargetPosition(-1900);
                return false;
            }
        }
        //allow the function to be able to be called from other files
        public Action extendSpec() {
            return new Extender.ExtendSpec();
        }

        public class ExtendGrab implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //when openclaw is run, set the claw to the open position
                extender.setTargetPosition(-850);
                return false;
            }
        }
        //allow the function to be able to be called from other files
        public Action extendGrab() {
            return new Extender.ExtendGrab();
        }

        public class UpdateExtender implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                //set the target position of the lift to 3000 ticks
                if (!extender.atTargetPosition()) {
                    extender.set(1);
                }
                return true;
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off2
            }
        }
        public Action updateExtender() {
            return new Extender.UpdateExtender();
        }
    }
                                                                                                                    //HERE
    public static class Macros {
        public Motor extender;
        public Motor leftPivot;
        public Motor rightPivot;
        private Servo wrist;
        private Servo claw;
        //create the claw object from hardware map

        public Macros(HardwareMap hardwareMap) {
            extender = new Motor(hardwareMap, "slide_drive", Motor.GoBILDA.RPM_312);
            extender.setRunMode(Motor.RunMode.PositionControl);
            extender.setPositionCoefficient(0.0025);
            //initialize our lift from hardwareMap
            leftPivot = new Motor(hardwareMap, "arm_drive_1", Motor.GoBILDA.RPM_60);
            //set the lift motor direction
            leftPivot.setInverted(true);
            //set position coefficient of the lift, (p value)
            //initialize our lift from hardwareMap
            rightPivot = new Motor(hardwareMap, "arm_drive_2", Motor.GoBILDA.RPM_60);
            //set the lift motor direction
            rightPivot.setInverted(false);
            rightPivot.setRunMode(Motor.RunMode.PositionControl);
            rightPivot.setPositionCoefficient(0.0016);
            leftPivot.setPositionCoefficient(0.0016);
            leftPivot.setRunMode(Motor.RunMode.PositionControl);
            wrist = hardwareMap.get(Servo.class, "claw_rotate");
            claw = hardwareMap.get(Servo.class, "claw");
        }

        //implement action class in our close claw function
        public class resetMacros implements Action {

            // actions are formatted via telemetry packets as below

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                //set the target position of the lift to 3000 ticks
                //leftPivot.resetEncoder();
                leftPivot.resetEncoder();
                rightPivot.resetEncoder();
                extender.resetEncoder();
                return false;
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off2
            }
        }
        public Action resetMacros() {
            return new Macros.resetMacros();
        }


        public class CloseGrabPosition implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //when closeclaw is run, set the claw to closed position
                extender.setTargetPosition(-500);
                leftPivot.setTargetPosition(-450);
                rightPivot.setTargetPosition(-450);
                wrist.setPosition(0.1);
                claw.setPosition(0.7);

                return false;
            }
        }
        //allow the function to be able to called from other files
        public Action closeGrabPosition() {
            return new Macros.CloseGrabPosition();
        }
        public class MiddleGrabPosition implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //when closeclaw is run, set the claw to closed position
                extender.setTargetPosition(-1650);
                leftPivot.setTargetPosition(-900);
                rightPivot.setTargetPosition(-900);
                wrist.setPosition(0.1);
                claw.setPosition(0.7);
                return false;
            }
        }
        //allow the function to be able to called from other files
        public Action middleGrabPosition() {
            return new Macros.MiddleGrabPosition();
        }
        public class FarGrabPosition implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //when closeclaw is run, set the claw to closed position
                extender.setTargetPosition(-2250);
                leftPivot.setTargetPosition(-950);
                rightPivot.setTargetPosition(-950);
                wrist.setPosition(0.1);
                claw.setPosition(0.7);
                return false;
            }
        }
        public Action farGrabPosition() {
            return new Macros.FarGrabPosition();
        }
        public class CloseGrab implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //when closeclaw is run, set the claw to closed position
                extender.setTargetPosition(-500);
                leftPivot.setTargetPosition(-350);
                rightPivot.setTargetPosition(-350);
                wrist.setPosition(0.1);
                claw.setPosition(0.5);

                return false;
            }
        }
        //allow the function to be able to called from other files
        public Action closeGrab() {
            return new Macros.CloseGrab();
        }
        public class MiddleGrab implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //when closeclaw is run, set the claw to closed position
                extender.setTargetPosition(-1650);
                leftPivot.setTargetPosition(-800);
                rightPivot.setTargetPosition(-800);
                wrist.setPosition(0.1);
                claw.setPosition(0.5);
                return false;
            }
        }
        //allow the function to be able to called from other files
        public Action middleGrab() {
            return new Macros.MiddleGrab();
        }
        public class FarGrab implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //when closeclaw is run, set the claw to closed position
                extender.setTargetPosition(-2250);
                leftPivot.setTargetPosition(-850);
                rightPivot.setTargetPosition(-850);
                wrist.setPosition(0.1);
                claw.setPosition(0.5);
                return false;
            }
        }
        public Action farGrab() {
            return new Macros.FarGrab();
        }
        public class PullIn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                extender.setTargetPosition(0);
                leftPivot.setTargetPosition(0);
                rightPivot.setTargetPosition(0);
                wrist.setPosition(0.65);
                claw.setPosition(0.5);
                return false;

            }
        }
        public Action pullIn() { return new Macros.PullIn(); }
        public class SampleUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                extender.setTargetPosition(-2200);
                leftPivot.setTargetPosition(-5100);
                rightPivot.setTargetPosition(-5100);
                wrist.setPosition(0);
                claw.setPosition(0.5);
                return false;

            }
        }
        public Action sampleUp() { return new Macros.SampleUp(); }
        public class SampleScore implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                extender.setTargetPosition(-2200);
                leftPivot.setTargetPosition(-5100);
                rightPivot.setTargetPosition(-5100);
                wrist.setPosition(0.65);
                claw.setPosition(0.7);
                return false;
            }
        }
        public Action sampleScore() { return new Macros.SampleScore(); }
        public class SampleReturn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                extender.setTargetPosition(0);
                leftPivot.setTargetPosition(0);
                rightPivot.setTargetPosition(0);
                wrist.setPosition(0.4);
                claw.setPosition(0.7);
                return false;
            }
        }
        public Action sampleReturn() { return new Macros.SampleReturn(); }

        public class UpdateMacros implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                //set the target position of the lift to 3000 ticks
                if (!extender.atTargetPosition()) {
                    extender.set(1);
                }
                if (!leftPivot.atTargetPosition()) {
                    leftPivot.set(0.9);
                }
                if (!rightPivot.atTargetPosition()) {
                    rightPivot.set(0.9);
                }
                return true;
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off2
            }
        }
        public Action updateMacros() {
            return new Macros.UpdateMacros();
        }


    }
}

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
                //when CloseClaw is run, set the claw to closed position
                claw.setPosition(0.55);
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
                claw.setPosition(1.0);
                return false;
            }
        }
        //allow the function to be able to be called from other files
        public Action openClaw() {
            return new Claw.OpenClaw();
        }
    }

    //lift class (this will require an encoder plugged into the motor)
    public static class Lift {
        private Motor lift;
        //create lift from hardwaremap and initialize it

        public Lift(HardwareMap hardwareMap) {
            //initialize our lift from hardwareMap
            lift = new Motor(hardwareMap, "lift", Motor.GoBILDA.RPM_30);
            //set the braking mode to brake when theres no power given so it better holds target position
            lift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            //put it into position control so it automatically flips direction
            lift.setRunMode(Motor.RunMode.PositionControl);
            //set the lift motor direction
            lift.setInverted(true);
            //set position coefficient of the lift, (p value)
            lift.setPositionCoefficient(0.001);
        }

        public class LiftUp implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;
            // actions are formatted via telemetry packets as below

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    lift.set(0.8);
                    initialized = true;
                }
                //set the target position of the lift to 3000 ticks
                lift.setTargetPosition(3000);
                if (!lift.atTargetPosition()) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun and stops the lift
                    lift.set(0);
                    return false;
                }
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off2
            }
        }
        public Action liftUp() {
            return new LiftUp();
        }

        public class LiftDown implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;
            // actions are formatted via telemetry packets as below

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //set the lifts target position to down position
                lift.setTargetPosition(10);
                // powers on motor, if it is not on
                if (!initialized) {
                    lift.set(-0.8);
                    initialized = true;
                }

                //if the lift isn't at the target position then repeat the loop
                if (!lift.atTargetPosition()) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun and stops the lift
                    lift.set(0);
                    return false;
                }
                // overall, the action powers the lift down until it goes below
                // 100 encoder ticks, then powers it off
            }
        }
        public Action liftDown(){
            return new LiftDown();
        }
    }
    public static class Arm {
        private Motor armMotor;
        //create arm from hardwareMap and initialize it

        public Arm(HardwareMap hardwareMap) {
            //initialize our arm from hardwareMap
            armMotor = new Motor(hardwareMap, "arm_motor", Motor.GoBILDA.RPM_117);
            //set the braking mode to brake when theres no power given so it better holds target position
            armMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            //put it into position control so it automatically flips direction
            armMotor.setRunMode(Motor.RunMode.PositionControl);
            //set the motor direction
            armMotor.setInverted(true);
            //set position coefficient of the lift, (p value)
            armMotor.setPositionCoefficient(0.001);
        }

        public class ArmUp implements Action {
            // checks if the arm motor has been powered on
            private boolean initialized = false;
            // actions are formatted via telemetry packets as below

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    armMotor.set(0.8);
                    initialized = true;
                }
                //set the target position of the lift to 3000 ticks
                armMotor.setTargetPosition(3000);
                if (!armMotor.atTargetPosition()) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun and stops the lift
                    armMotor.set(0);
                    return false;
                }
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off2
            }
        }
        public Action ArmUp() {
            return new ArmUp();
        }

        public class ArmDown implements Action {
            // checks if the arm motor has been powered on
            private boolean initialized = false;
            // actions are formatted via telemetry packets as below

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //set the lifts target position to down position
                armMotor.setTargetPosition(10);
                // powers on motor, if it is not on
                if (!initialized) {
                    armMotor.set(-0.8);
                    initialized = true;
                }

                //if the arm isn't at the target position then repeat the loop
                if (!armMotor.atTargetPosition()) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun and stops the arm
                    armMotor.set(0);
                    return false;
                }
                // overall, the action powers the arm down until it goes below
                // 100 encoder ticks, then powers it off
            }
        }
        public Action ArmDown(){
            return new ArmDown();
        }
    }

    public static class Slide {
        private Motor slideMotor;
        //create lift from hardwaremap and initialize it

        public Slide(HardwareMap hardwareMap) {
            //initialize our lift from hardwareMap
            slideMotor = new Motor(hardwareMap, "slide_motor", Motor.GoBILDA.RPM_117);
            //set the braking mode to brake when theres no power given so it better holds target position
            slideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            //put it into position control so it automatically flips direction
            slideMotor.setRunMode(Motor.RunMode.PositionControl);
            //set the lift motor direction
            slideMotor.setInverted(true);
            //set position coefficient of the lift, (p value)
            slideMotor.setPositionCoefficient(0.001);
        }

        public class LiftUp implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;
            // actions are formatted via telemetry packets as below

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    slideMotor.set(0.8);
                    initialized = true;
                }
                //set the target position of the lift to 3000 ticks
                slideMotor.setTargetPosition(3000);
                if (!slideMotor.atTargetPosition()) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun and stops the lift
                    slideMotor.set(0);
                    return false;
                }
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off2
            }
        }
        public Action liftUp() {
            return new LiftUp();
        }

        public class LiftDown implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;
            // actions are formatted via telemetry packets as below

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //set the lifts target position to down position
                slideMotor.setTargetPosition(10);
                // powers on motor, if it is not on
                if (!initialized) {
                    slideMotor.set(-0.8);
                    initialized = true;
                }

                //if the lift isn't at the target position then repeat the loop
                if (!slideMotor.atTargetPosition()) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun and stops the lift
                    slideMotor.set(0);
                    return false;
                }
                // overall, the action powers the lift down until it goes below
                // 100 encoder ticks, then powers it off
            }
        }
        public Action liftDown(){
            return new LiftDown();
        }
    }
    public static class Intake {
        private Motor intake;
        //create the claw object from hardware map

        public Intake(HardwareMap hardwareMap) {
            //initialize our intake from hardwareMap
            intake = new Motor(hardwareMap, "intake", Motor.GoBILDA.RPM_435);
            //set the braking mode to float when theres no power given so it doesn't do anything
            intake.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
            //set the runmode to raw power
            intake.setRunMode(Motor.RunMode.RawPower);
            //set the direction of the motor
            intake.setInverted(false);
        }

        //implement action class in our spin intake forward function.

        public class spinForward implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //when intake spinforward is run, spin the intake forward
                intake.set(0.8);
                return false;
            }
        }
        //allow the function to be able to called from other files
        public Action spinForward() {
            return new Intake.spinForward();
        }
        //create an spin backward function by implementing action class

        public class spinBackward implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //when spin backward is run, spin the intake backwards
                intake.set(-0.8);
                return false;
            }
        }
        //allow the function to be able to be called from other files
        public Action spinBackward() {
            return new Intake.spinBackward();
        }
    }
}

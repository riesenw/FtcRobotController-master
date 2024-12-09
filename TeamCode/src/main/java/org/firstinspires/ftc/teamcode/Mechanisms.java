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
        final double CLAW_CLOSED_POSITION = 0.55;
        final double CLAW_OPEN_POSITION = 1.0;
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
                claw.setPosition(CLAW_CLOSED_POSITION);
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
                claw.setPosition(CLAW_OPEN_POSITION);
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

        public Action liftDown() {
            return new LiftDown();
        }
    }

    public static class Arm {
        final int ARM_RAISED_POSITION = -2600;
        final int ARM_LOWERED_POSITION = 0;
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
                //set the target position of the lift to ARM_RAISED_POSITION ticks
                armMotor.setTargetPosition(ARM_RAISED_POSITION);
                if (!armMotor.atTargetPosition()) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun and stops the lift
                    armMotor.set(0);
                    return false;
                }
                // overall, the action powers the arm until it surpasses
                // ARM_RAISED_POSITION encoder ticks, then powers it off
            }
        }

        public Action armUp() {
            return new ArmUp();
        }

        public class ArmDown implements Action {
            // checks if the arm motor has been powered on
            private boolean initialized = false;
            // actions are formatted via telemetry packets as below

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //set the arm target position to down position
                armMotor.setTargetPosition(ARM_LOWERED_POSITION);
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
                // ARM_RAISED_POSITION encoder ticks, then powers it off
            }
        }

        public Action armDown() {
            return new ArmDown();
        }
    }

    public static class Slide {
        final int SLIDE_EXTENDED_POSITION = -2200;
        final int SLIDE_RETRACTED_POSITION = 0;
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

        public class ExtendSlide implements Action {
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
                //set the target position of the lift to SLIDE_EXTENDED_POSITION ticks
                slideMotor.setTargetPosition(SLIDE_EXTENDED_POSITION);
                if (!slideMotor.atTargetPosition()) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun and stops the slide
                    slideMotor.set(0);
                    return false;
                }
                // overall, the action powers the lift until it surpasses
                // SLIDE_EXTENDED_POSITION encoder ticks, then powers it off2
            }
        }

        public Action extendSlide() {
            return new ExtendSlide();
        }

        public class RetractSlide implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;
            // actions are formatted via telemetry packets as below

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //set the slide's target position to retracted position
                slideMotor.setTargetPosition(SLIDE_RETRACTED_POSITION);
                // powers on motor, if it is not on
                if (!initialized) {
                    slideMotor.set(-0.8);
                    initialized = true;
                }

                //if the slide isn't at the target position then repeat the loop
                if (!slideMotor.atTargetPosition()) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun and stops the lift
                    slideMotor.set(0);
                    return false;
                }
                // overall, the action powers the slide down until it goes below
                // SLIDE_RETRACTED_POSITION encoder ticks, then powers it off
            }
        }

        public Action retractSlide() {
            return new RetractSlide();
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

    public static class Wrist {
        final double WRIST_DOWN_POSITION = 0.0;
        final double WRIST_MID_POSITION = 0.325;
        final double WRIST_UP_POSITION = 0.65;

        private Servo wristServo;
        //create the Wrist object from hardware map

        public Wrist(HardwareMap hardwareMap) {
            wristServo = hardwareMap.get(Servo.class, "wrist_servo");
        }

        public class WristDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //when wristDown is run, set the wrist to down position
                wristServo.setPosition(WRIST_DOWN_POSITION);
                return false;
            }
        }

        //allow the function to be able to called from other files
        public Action wristDown() {
            return new Wrist.WristDown();
        }
        //create an wristUp function by implementing action class

        public class wristUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //when wristUp is run, set the wrist to the up position
                wristServo.setPosition(WRIST_UP_POSITION);
                return false;
            }
        }

        //allow the function to be able to be called from other files
        public Action wristUp() {
            return new Wrist.wristUp();
        }

        public class wristMid implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //when wristUp is run, set the wrist to the mid position
                wristServo.setPosition(WRIST_MID_POSITION);
                return false;
            }
        }

    }
}



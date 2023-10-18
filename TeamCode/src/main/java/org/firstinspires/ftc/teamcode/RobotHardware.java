package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class RobotHardware {

    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor flipperArm = null;
    private DcMotor sweeper = null;
    private Servo pixelRelease = null;
    public ColorRangeSensor distanceSensor = null;

    private IMU imu         = null;

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    public static final double PIXEL_CARRY_POSITION = 0.0;
    public static final double PIXEL_RELEASE_POSITION = 0.5;
    public static final int PICKUP_POSITION = 0;
    public static final int CARRY_POSITION = 100;
    public static final int FLIP_POSITION = 500;
    public static final double SERVO_SPEED = 0.02;  // sets rate to move servo
    public static final double ARM_UP_POWER = 0.45;
    public static final double ARM_DOWN_POWER = -0.45;
    public double MAX_DRIVE_SPEED = 0.5;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double COUNTS_PER_MOTOR_REV = 537.7;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init() {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        leftFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_back_drive");
        flipperArm = myOpMode.hardwareMap.get(DcMotor.class, "flipper_arm");
        sweeper = myOpMode.hardwareMap.get(DcMotor.class, "sweeper");
        distanceSensor = myOpMode.hardwareMap.get(ColorRangeSensor.class, "distance_sensor");

        imu = myOpMode.hardwareMap.get(IMU.class, "imu");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sweeper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flipperArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flipperArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pixelRelease = myOpMode.hardwareMap.get(Servo.class, "pixel_release");
        pixelRelease.setPosition(PIXEL_CARRY_POSITION);

        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         *
         * To Do:  EDIT these two lines to match YOUR mounting configuration.
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

//    /**
//     * Calculates the left/right motor powers required to achieve the requested
//     * robot motions: Drive (Axial motion) and Turn (Yaw motion).
//     * Then sends these power levels to the motors.
//     *
//     * @param Drive Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
//     * @param Turn  Right/Left turning power (-1.0 to 1.0) +ve is CW
//     */
//    public void driveRobot(double Drive, double Turn) {
//        // Combine drive and turn for blended motion.
//        double left = Drive + Turn;
//        double right = Drive - Turn;
//
//        // Scale the values so neither exceed +/- 1.0
//        double max = Math.max(Math.abs(left), Math.abs(right));
//        if (max > 1.0) {
//            left /= max;
//            right /= max;
//        }
//
//        // Use existing function to drive both wheels.
//        setDrivePower(left, right);
//    }

//    /**
//     * Pass the requested wheel motor powers to the appropriate hardware drive motors.
//     *
//     * @param leftWheel  Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
//     * @param rightWheel Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
//     */
//    public void setDrivePower(double leftWheel, double rightWheel) {
//        // Output the values to the motor drives.
//        leftDrive.setPower(leftWheel);
//        rightDrive.setPower(rightWheel);
//    }

    public void moveRobot(double axial, double lateral, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower = axial - lateral - yaw;
        double rightFrontPower = axial + lateral + yaw;
        double leftBackPower = axial + lateral - yaw;
        double rightBackPower = axial - lateral + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    public void setArmPower(double power) {
        flipperArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flipperArm.setPower(power);
    }

    public void setArmPosition(int targetPosition) {
        flipperArm.setTargetPosition(targetPosition);
        flipperArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void deployPixel() {
        pixelRelease.setPosition(PIXEL_RELEASE_POSITION);
    }

    public boolean isMoving() {
        return leftFrontDrive.isBusy() ||
                rightFrontDrive.isBusy() ||
                leftBackDrive.isBusy() ||
                rightBackDrive.isBusy();
    }

//    public void setSweeperOn(boolean on) {
//        if (on) {
//            double SWEEPER_POWER = 0.5;
//            sweeper.setPower(SWEEPER_POWER);
//        } else {
//            sweeper.setPower(0.0);
//        }
//    }

    public void setSweeperPower(double power){
        sweeper.setPower(power);
    }

    public void raiseArm() {
        if (flipperArm.getTargetPosition() == PICKUP_POSITION) {
            flipperArm.setTargetPosition(CARRY_POSITION);
            flipperArm.setPower(0.2);
        } else if (flipperArm.getTargetPosition() == CARRY_POSITION) {
            flipperArm.setTargetPosition(FLIP_POSITION);
            flipperArm.setPower(0.75);
        }
    }

    public void lowerArm() {
        if (flipperArm.getTargetPosition() == FLIP_POSITION) {
            flipperArm.setTargetPosition(CARRY_POSITION);
            flipperArm.setPower(0.75);
        } else if (flipperArm.getTargetPosition() == CARRY_POSITION) {
            flipperArm.setTargetPosition(PICKUP_POSITION);
            flipperArm.setPower(0.2);
        }
    }

    public boolean driveStraight(double maxDriveSpeed,
                                 double distance) {

        // This method uses IMU to go straight forward or back from current heading:
        // robot must be pointed in the correct direction prior to calling.
        double heading = getHeading();
        boolean hitSomething = false;

        if (myOpMode.opModeIsActive()) {


            // Determine new target position, and pass to motor controller
            int moveCounts = (int) (distance * COUNTS_PER_INCH);
            leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFrontDrive.setTargetPosition(moveCounts);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontDrive.setTargetPosition(moveCounts);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackDrive.setTargetPosition(moveCounts);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFrontDrive.setTargetPosition(moveCounts);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0, 0);

            // keep looping while we are still active, and ALL motors are running.
            while (myOpMode.opModeIsActive() &&
                    leftFrontDrive.isBusy() &&
                    rightFrontDrive.isBusy() &&
                    leftBackDrive.isBusy() &&
                    rightBackDrive.isBusy()) {

//                if(touchSensor.isPressed()){
//                    hitSomething = true;
//                }

                // Determine required steering to keep on heading
                double turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(maxDriveSpeed,0.0,turnSpeed);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0,0);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        return hitSomething;
    }

    /**
     * Use a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading   The desired absolute heading (relative to last heading reset)
     * @param proportionalGain Gain factor applied to heading error to obtain turning power.
     * @return Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        double targetHeading = desiredHeading;

        double headingError = targetHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * read the Robot heading directly from the IMU (in degrees)
     */
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}

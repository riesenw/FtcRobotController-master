package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class RobotHardware {

    //  This section initializes variables and constants pertaining to camera-based navigation.
    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 6.0; //  this is how close the camera should get to the target (inches)
    final double ARRIVAL_TOLERANCE = 1.0; // margin of error for determining arrival at destination (inches)
    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN = 0.002;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN = 0.002;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)


    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor arm = null;
    private DcMotor sweeper = null;
    private Servo pixelRelease = null;
    private Servo launchAirPlane = null;
    private ColorRangeSensor distanceSensor = null;
    private IMU imu = null;


    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    public static final double PIXEL_CARRY_POSITION = 0.0;
    public static final double PIXEL_RELEASE_POSITION = 10;

    public static final double AIR_PLANE_CARRY_POSITION = 0.0;
    public static final double AIR_PLANE_LAUNCH_POSITION = 0.30;

    public static final int PICKUP_POSITION = 75;
    public static final int CARRY_POSITION = 0;
    public static final int FLIP_POSITION = -200;
    //    public double maxDriveSpeed = 0.1;
    private static final double STEERING_CORRECTION = 0.01;
    static final double COUNTS_PER_INCH_DRIVING = 31;
    static final double COUNTS_PER_INCH_STRAFING = 37;
    //    static final double AXIAL_REDUCTION_FACTOR = 0.3;
    static final double ODD_WHEEL_MULTIPLIER = 1.0; // 2.67;  //corrects for one wheel having different gear ratio

    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double P_TURN_GAIN = 0.003;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_GAIN = 0.003;     // Larger is more responsive, but also less stable

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
        arm = myOpMode.hardwareMap.get(DcMotor.class, "flipper_arm");
        sweeper = myOpMode.hardwareMap.get(DcMotor.class, "sweeper");

        pixelRelease = myOpMode.hardwareMap.get(Servo.class, "pixel_release");
        launchAirPlane = myOpMode.hardwareMap.get(Servo.class, "drone_launch");

        distanceSensor = myOpMode.hardwareMap.get(ColorRangeSensor.class, "distance_sensor");

        imu = myOpMode.hardwareMap.get(IMU.class, "imu");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sweeper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setTargetPosition(CARRY_POSITION);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pixelRelease.setPosition(PIXEL_CARRY_POSITION);

        launchAirPlane.setPosition(AIR_PLANE_CARRY_POSITION);


        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         *
         * To Do:  EDIT these two lines to match YOUR mounting configuration.
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.DOWN;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        // Initialize the Apriltag Detection process
        initAprilTag();

        if (USE_WEBCAM)
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    public void moveRobot(double axial, double lateral, double yaw) {
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        double correctedAxial = axial * AXIAL_REDUCTION_FACTOR;

        // Calculate wheel powers.
        double leftFrontPower = axial - lateral - yaw;
        double rightFrontPower = axial + lateral + yaw;
        double leftBackPower = axial + lateral - yaw;
        double rightBackPower = axial - lateral + yaw;

        // Normalize wheel powers to be less than maxDriveSpeed
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
        rightFrontDrive.setPower(rightFrontPower * ODD_WHEEL_MULTIPLIER);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    public void changeArmPosition(int targetPosition) {
        arm.setTargetPosition(targetPosition);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void deployPixel() {
        pixelRelease.setPosition(PIXEL_RELEASE_POSITION);
        myOpMode.sleep(250);
        pixelRelease.setPosition(PIXEL_CARRY_POSITION);  // reset for next match
    }

    public void lauchAirPlane() {
        launchAirPlane.setPosition(AIR_PLANE_LAUNCH_POSITION);
        myOpMode.sleep(500);
        launchAirPlane.setPosition(AIR_PLANE_CARRY_POSITION); // reset for next launch
    }

    public void setSweeperOn(boolean on) {
        if (on) {
            double SWEEPER_POWER = -0.5;
            sweeper.setPower(SWEEPER_POWER);
        } else {
            sweeper.setPower(0.0);
        }
    }

    public void setUnjamSweeperOn(boolean on) {
        if (on) {
            double SWEEPER_POWER = 0.5;
            sweeper.setPower(SWEEPER_POWER);
        } else {
            sweeper.setPower(0.0);
        }
    }


    public void setSweeperPower(double power) {
        sweeper.setPower(power);
    }

    public void moveArmToPickupPosition() {
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        arm.setTargetPosition(PICKUP_POSITION);
        arm.setPower(0.1);
    }

    public void moveArmToCarryPosition() {
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setTargetPosition(CARRY_POSITION);
        arm.setPower(0.1);
    }

    public void moveArmToFlipPosition() {
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        arm.setTargetPosition(FLIP_POSITION);
        arm.setPower(0.1);
    }

    public boolean driveStraight(double maxDriveSpeed,
                                 double distance) throws InterruptedException {

        // This method uses IMU to go straight forward or back from current heading:
        // robot must be pointed in the correct direction prior to calling.
        double heading = getHeading();
        boolean hitSomething = false;

        if (myOpMode.opModeIsActive()) {


            // Determine new target position, and pass to motor controller
            int moveCounts = (int) (distance * COUNTS_PER_INCH_DRIVING);
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
            leftFrontDrive.setPower(maxDriveSpeed);
            rightFrontDrive.setPower(maxDriveSpeed);
            leftBackDrive.setPower(maxDriveSpeed);
            rightBackDrive.setPower(maxDriveSpeed);

            while (myOpMode.opModeIsActive()) {
                Thread.sleep(20);
            }

//            // keep looping while we are still active, and ALL motors are running.
//            while (myOpMode.opModeIsActive() &&
//                    leftFrontDrive.isBusy() &&
//                    rightFrontDrive.isBusy() &&
//                    leftBackDrive.isBusy() &&
//                    rightBackDrive.isBusy()) {
//
////                if(touchSensor.isPressed()){
////                    hitSomething = true;
////                }
//
//                // Determine required steering to keep on heading
//                double turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);
//
//                // if driving in reverse, the motor correction also needs to be reversed
//                if (distance < 0)
//                    turnSpeed *= -1.0;
//
//                // Apply the turning correction to the current driving speed.
//                moveRobot(maxDriveSpeed,0.0,turnSpeed);
//            }
//
//            // Stop all motion & Turn off RUN_TO_POSITION
//            moveRobot(0, 0,0);
//            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

    private double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        double currentHeading = getHeading();
        myOpMode.telemetry.addData("Heading:", currentHeading);
        myOpMode.telemetry.update();
        double headingError = desiredHeading - currentHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * read the Robot heading directly from the IMU (in degrees)
     */
    private double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    public boolean searchForward(double heading, double distance, double speed) {
        boolean obstacleDetected = false;
        rotateToHeading(heading);

        leftFrontDrive.setTargetPosition((int) (distance * COUNTS_PER_INCH_DRIVING));
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setPower(speed);
        rightFrontDrive.setTargetPosition((int) (distance * COUNTS_PER_INCH_DRIVING));
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setPower(speed);
        leftBackDrive.setTargetPosition((int) (distance * COUNTS_PER_INCH_DRIVING));
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setPower(speed);
        rightBackDrive.setTargetPosition((int) (distance * COUNTS_PER_INCH_DRIVING));
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setPower(speed);

        while (leftFrontDrive.isBusy() && myOpMode.opModeIsActive()) {
            double steeringCorrection = getSteeringCorrection(heading, P_DRIVE_GAIN);
            leftFrontDrive.setPower(speed + steeringCorrection);
            rightFrontDrive.setPower(speed - steeringCorrection);
            leftBackDrive.setPower(speed + steeringCorrection);
            rightBackDrive.setPower(speed - steeringCorrection);
            obstacleDetected = checkForObstacle();
            myOpMode.sleep(20);
        }
        return obstacleDetected;
    }

    public void backUp(double directionToMove, double distance, double speed) {

        // since we are backing up, direction moving is given but direction facing is opposite.
        double heading = directionToMove - 180;
        // Normalize the error to be within +/- 180 degrees
        while (directionToMove > 180) directionToMove -= 360;
        while (directionToMove <= -180) directionToMove += 360;
        rotateToHeading(heading);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setTargetPosition(-(int) (distance * COUNTS_PER_INCH_DRIVING));
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setPower(speed);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setTargetPosition(-(int) (distance *
                COUNTS_PER_INCH_DRIVING * ODD_WHEEL_MULTIPLIER));
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setPower(speed);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setTargetPosition(-(int) (distance * COUNTS_PER_INCH_DRIVING));
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setPower(speed);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setTargetPosition(-(int) (distance * COUNTS_PER_INCH_DRIVING));
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setPower(speed);

        while (leftFrontDrive.isBusy() && myOpMode.opModeIsActive()) {
            double steeringCorrection = getSteeringCorrection(heading, P_DRIVE_GAIN);
            leftFrontDrive.setPower(speed + steeringCorrection);
            rightFrontDrive.setPower((speed - steeringCorrection) * ODD_WHEEL_MULTIPLIER);
            leftBackDrive.setPower(speed + steeringCorrection);
            rightBackDrive.setPower(speed - steeringCorrection);
            myOpMode.sleep(20);
        }
    }

    public boolean forward(double directionToMove, double distance, double speed) {

        // Normalize the travel direction given to be within +/- 180 degrees
        while (directionToMove > 180) directionToMove -= 360;
        while (directionToMove <= -180) directionToMove += 360;
        double heading = directionToMove;

        boolean objectDetected = false;

//        rotateToHeading(heading);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setTargetPosition((int) (distance * COUNTS_PER_INCH_DRIVING));
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setPower(speed);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setTargetPosition((int) (distance * COUNTS_PER_INCH_DRIVING * ODD_WHEEL_MULTIPLIER));
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setPower(speed * ODD_WHEEL_MULTIPLIER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setTargetPosition((int) (distance * COUNTS_PER_INCH_DRIVING));
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setPower(speed);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setTargetPosition((int) (distance * COUNTS_PER_INCH_DRIVING));
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setPower(speed);

        while (leftFrontDrive.isBusy() && myOpMode.opModeIsActive()) {
            double steeringCorrection = getSteeringCorrection(heading, P_DRIVE_GAIN);
            leftFrontDrive.setPower(speed - steeringCorrection);
            rightFrontDrive.setPower((speed + steeringCorrection) * ODD_WHEEL_MULTIPLIER);
            leftBackDrive.setPower(speed - steeringCorrection);
            rightBackDrive.setPower(speed + steeringCorrection);

            double objectDistance = distanceSensor.getDistance(DistanceUnit.INCH);
            if (Double.isFinite(objectDistance) && objectDistance < 3.0) {
                objectDetected = true;
            }
            ;

            myOpMode.telemetry.addData("Object detected?:", objectDetected);
            myOpMode.telemetry.update();

            myOpMode.sleep(20);
        }
        return objectDetected;
    }

    public void holdHeading(double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        double maxTurnSpeed = 0.2;

        // keep looping while we have time remaining.
        while (myOpMode.opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            double turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, 0, turnSpeed);
            myOpMode.telemetry.addData("Timer: ", holdTimer.time());
            myOpMode.telemetry.update();
        }

        // Stop all motion;
        moveRobot(0, 0, 0);
        myOpMode.telemetry.addLine("TImer done.");
        myOpMode.telemetry.update();
    }

    public void strafeLeft(double directionToMove, double distance, double speed) {

        // since we are strafing left, the heading (direction we need to face)
        // is 90 degrees to the RIGHT of the direction of motion, which is given
        // in the argument
        double heading = directionToMove + 90;
        // Normalize the heading to be within +/- 180 degrees
        while (directionToMove > 180) directionToMove -= 360;
        while (directionToMove <= -180) directionToMove += 360;
//        rotateToHeading(heading);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setTargetPosition((int) (-distance * COUNTS_PER_INCH_STRAFING));
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setPower(speed);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setTargetPosition((int) (distance *
                COUNTS_PER_INCH_STRAFING * ODD_WHEEL_MULTIPLIER));
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setPower(speed * ODD_WHEEL_MULTIPLIER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setTargetPosition((int) (distance * COUNTS_PER_INCH_STRAFING));
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setPower(speed);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setTargetPosition((int) (-distance * COUNTS_PER_INCH_STRAFING));
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setPower(speed);

        while (leftFrontDrive.isBusy() && myOpMode.opModeIsActive()) {
            double steeringCorrection = getSteeringCorrection(heading, P_DRIVE_GAIN);
            steeringCorrection = 0; //steeringCorrection not working right-- fix later.
            leftFrontDrive.setPower(speed + steeringCorrection);
            rightFrontDrive.setPower((speed - steeringCorrection) * ODD_WHEEL_MULTIPLIER);
            leftBackDrive.setPower(speed + steeringCorrection);
            rightBackDrive.setPower(speed - steeringCorrection);
            myOpMode.sleep(20);
        }

    }

    public void strafeRight(double directionToMove, double distance, double speed) {

        // since we are strafing right, the heading (direction we need to face)
        // is 90 degrees to the LEFT of the direction of motion, which is given
        // in the argument
        double heading = directionToMove - 90;
        // Normalize the heading to be within +/- 180 degrees
        while (directionToMove > 180) directionToMove -= 360;
        while (directionToMove <= -180) directionToMove += 360;
//        rotateToHeading(heading);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setTargetPosition((int) (distance * COUNTS_PER_INCH_STRAFING));
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setPower(speed);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setTargetPosition((int) (-distance *
                COUNTS_PER_INCH_STRAFING * ODD_WHEEL_MULTIPLIER));
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setPower(speed * ODD_WHEEL_MULTIPLIER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setTargetPosition((int) (-distance * COUNTS_PER_INCH_STRAFING));
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setPower(speed);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setTargetPosition((int) (distance * COUNTS_PER_INCH_STRAFING));
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setPower(speed);

        while (leftFrontDrive.isBusy() && myOpMode.opModeIsActive()) {
            double steeringCorrection = getSteeringCorrection(heading, P_DRIVE_GAIN);
            steeringCorrection = 0; //steeringCorrection not working right-- fix later.
            leftFrontDrive.setPower(speed - steeringCorrection);
            rightFrontDrive.setPower((speed + steeringCorrection) * ODD_WHEEL_MULTIPLIER);
            leftBackDrive.setPower(speed - steeringCorrection);
            rightBackDrive.setPower(speed + steeringCorrection);
            myOpMode.sleep(20);
        }

    }

    public void rotateToHeading(double heading) {
        double steeringCorrection = getSteeringCorrection(heading, P_TURN_GAIN);
        myOpMode.telemetry.addData("Steering Correction:", steeringCorrection);
        myOpMode.telemetry.update();
        myOpMode.sleep(500);

        while (myOpMode.opModeIsActive() && Math.abs(steeringCorrection) > 0.01) {
            moveRobot(0, 0, steeringCorrection);
            myOpMode.sleep(20);
            steeringCorrection = getSteeringCorrection(heading, P_TURN_GAIN);
            myOpMode.telemetry.addData("Steering Correction:", steeringCorrection);
            myOpMode.telemetry.update();
        }
        moveRobot(0, 0, 0);
    }

    private boolean checkForObstacle() {
        double distanceSensorReading = distanceSensor.getDistance(DistanceUnit.INCH);
        return distanceSensorReading < 3.0;
    }

    public void placePixelOnBoard() {
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (distanceSensor.getDistance(DistanceUnit.INCH) > 6.0) {
            leftFrontDrive.setPower(0.2);
            leftBackDrive.setPower(0.2);
            rightFrontDrive.setPower(0.2);
            rightBackDrive.setPower(0.2);
        }
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        moveArmToFlipPosition();
        myOpMode.sleep(4000);
    }

    public void basicForward(double distance, double speed) {

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setTargetPosition((int) (distance / COUNTS_PER_INCH_DRIVING));
        leftBackDrive.setTargetPosition((int) (distance / COUNTS_PER_INCH_DRIVING));
        rightFrontDrive.setTargetPosition((int) (distance / COUNTS_PER_INCH_DRIVING));
        rightBackDrive.setTargetPosition((int) (distance / COUNTS_PER_INCH_DRIVING));

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFrontDrive.setPower(0.2);
        leftBackDrive.setPower(0.2);
        rightFrontDrive.setPower(0.2);
        rightBackDrive.setPower(0.2);

//        while (leftFrontDrive.isBusy();
//
//        leftFrontDrive.setPower(0);
//        leftBackDrive.setPower(0);
//        rightFrontDrive.setPower(0);
//        rightBackDrive.setPower(0);
//

    }

    public boolean autoDriveToTarget(int target_ID) {
        boolean targetFound = false;    // Set to true when an AprilTag target is detected
        double drive = 0;               // Desired forward power/speed (-1 to +1)
        double strafe = 0;               // Desired strafe power/speed (-1 to +1)
        double turn = 0;                  // Desired turning power/speed (-1 to +1)
        boolean targetReached = false;
        boolean targetLost = false;

        while (myOpMode.opModeIsActive() && !targetLost && !targetReached) {
            targetFound = false;
            desiredTag = null;

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if (detection.id == target_ID) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    }
                }
            }

            if (targetFound) {

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                myOpMode.telemetry.addData("Distance to target:",rangeError);
                myOpMode.telemetry.update();
                targetReached = Math.abs(rangeError) < ARRIVAL_TOLERANCE;
                double headingError = desiredTag.ftcPose.bearing;
                double yawError = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            } else {

                // target lost: exit method

                targetLost = true;
            }

            // Apply desired axes motions to the drivetrain.
            moveRobot(drive, strafe, turn);
            myOpMode.sleep(10);
        }
        drive = 0;
        strafe = 0;
        turn = 0;
        moveRobot(drive, strafe, turn);
        return targetReached;
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }


    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            myOpMode.telemetry.addData("Camera", "Waiting");
            myOpMode.telemetry.update();
            while (!myOpMode.isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                myOpMode.sleep(20);
            }
            myOpMode.telemetry.addData("Camera", "Ready");
            myOpMode.telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!myOpMode.isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                myOpMode.sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            myOpMode.sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            myOpMode.sleep(20);
        }
    }
}

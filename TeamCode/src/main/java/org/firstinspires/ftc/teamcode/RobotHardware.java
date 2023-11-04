package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class RobotHardware {


    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor arm = null;
    private DcMotor sweeper = null;
    private Servo pixelRelease = null;
    private ColorRangeSensor distanceSensor = null;
    private IMU imu = null;

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    public static final double PIXEL_CARRY_POSITION = 0.0;
    public static final double PIXEL_RELEASE_POSITION = 10;
    public static final int PICKUP_POSITION = 75;
    public static final int CARRY_POSITION = 0;
    public static final int FLIP_POSITION = -200;
//    public double maxDriveSpeed = 0.1;
    private static final double STEERING_CORRECTION = 0.01;
    static final double COUNTS_PER_INCH_DRIVING = 31;
    static final double COUNTS_PER_INCH_STRAFING = 1;
//    static final double AXIAL_REDUCTION_FACTOR = 0.3;
    static final double ODD_WHEEL_MULTIPLIER = 2.67;  //corrects for one wheel having different gear ratio

    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double P_TURN_GAIN = 0.02;     // Larger is more responsive, but also less stable
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

        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         *
         * To Do:  EDIT these two lines to match YOUR mounting configuration.
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

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


    public void setSweeperOn(boolean on) {
        if (on) {
            double SWEEPER_POWER = -0.5;
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
        rightFrontDrive.setTargetPosition(-(int) (distance * COUNTS_PER_INCH_DRIVING));
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
            rightFrontDrive.setPower(speed - steeringCorrection);
            leftBackDrive.setPower(speed + steeringCorrection);
            rightBackDrive.setPower(speed - steeringCorrection);
            myOpMode.sleep(20);
        }
    }

    public void forward(double directionToMove, double distance, double speed) {

        // Normalize the travel direction given to be within +/- 180 degrees
        while (directionToMove > 180) directionToMove -= 360;
        while (directionToMove <= -180) directionToMove += 360;
        double heading = directionToMove;

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
            myOpMode.sleep(20);
        }
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
        rotateToHeading(heading);

        leftFrontDrive.setTargetPosition((int) (distance * COUNTS_PER_INCH_STRAFING));
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setPower(speed);
        rightFrontDrive.setTargetPosition((int) (distance * COUNTS_PER_INCH_STRAFING));
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setPower(speed);
        leftBackDrive.setTargetPosition((int) (distance * COUNTS_PER_INCH_STRAFING));
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setPower(speed);
        rightBackDrive.setTargetPosition((int) (distance * COUNTS_PER_INCH_STRAFING));
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setPower(speed);

        while (leftFrontDrive.isBusy() && myOpMode.opModeIsActive()) {
            double steeringCorrection = getSteeringCorrection(heading, P_DRIVE_GAIN);
            leftFrontDrive.setPower(speed + steeringCorrection);
            rightFrontDrive.setPower(speed - steeringCorrection);
            leftBackDrive.setPower(speed + steeringCorrection);
            rightBackDrive.setPower(speed - steeringCorrection);
            myOpMode.sleep(20);
        }

    }

    public void rotateToHeading(double heading) {
        double steeringCorrection = getSteeringCorrection(heading, P_TURN_GAIN);
        myOpMode.telemetry.addData("Steering Correction:", steeringCorrection);
        myOpMode.telemetry.update();
        myOpMode.sleep(500);

        while (myOpMode.opModeIsActive() && steeringCorrection > 0.1) {
            moveRobot(0, 0, steeringCorrection);
            myOpMode.sleep(20);
            steeringCorrection = getSteeringCorrection(heading, P_TURN_GAIN);
            myOpMode.telemetry.addData("Steering Correction:", steeringCorrection);
            myOpMode.telemetry.update();
        }
        moveRobot(0,0,0);
    }

    private boolean checkForObstacle() {
        double distanceSensorReading = distanceSensor.getDistance(DistanceUnit.INCH);
        return distanceSensorReading < 3.0;
    }
}

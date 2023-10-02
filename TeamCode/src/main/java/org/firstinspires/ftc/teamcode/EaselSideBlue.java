/* FTC Team Code #6025 Team "Low Battery" 2023-2024
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Arrays;
import java.util.Iterator;
import java.util.List;
import java.util.OptionalDouble;

@Autonomous(name = "Easel Side Blue")

public class EaselSideBlue extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    private enum propPosition {RIGHT, LEFT, CENTER, NONE}

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private static final double SEARCH_ANGLE = 10;
    private static final double SEARCH_ANGLE_TOLERANCE = 2;
    private static final double STANDOFF_DISTANCE = 6;
    private static final List<Double> searchHeadingsList = Arrays.asList(0.0, 10.0, -10.0);
    private static final int PROP_TAG_ID = 1;
    IMU imu;

    @Override
    public void runOpMode() {

        initAprilTag();
        initImu();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        waitForStart();
        Enum<propPosition> propPosition = locateTag(PROP_TAG_ID);
        driveDirectlyToTag(PROP_TAG_ID);
        driveForwardDistance(STANDOFF_DISTANCE);
    }

    public void driveDirectlyToTag(int tagId) {
        OptionalDouble range;
        do {
            range = getRangeToTag(tagId);
            telemetry.addData("Range:", range);
            telemetry.update();
            sleep(20);
        } while (opModeIsActive() && range.orElse(10) > STANDOFF_DISTANCE);
        telemetry.addLine("Target Tag reached!");
        telemetry.update();
    }

    private void initAprilTag() {
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }

    }

    private void initImu() {
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                        )
                )
        );
    }

    private Enum<propPosition> locateTag(int tagID) {
        OptionalDouble optionalBearing = getBearingToTag(tagID);
        Iterator<Double> iterator = searchHeadingsList.iterator();
        while (opModeIsActive() && (!optionalBearing.isPresent() || optionalBearing.getAsDouble() > SEARCH_ANGLE_TOLERANCE)) {
            if (!optionalBearing.isPresent() && iterator.hasNext()) {
                rotateRobot(iterator.next());
            } else {
                rotateRobot(optionalBearing.getAsDouble());
            }
        }
        if (!optionalBearing.isPresent()) {
            return propPosition.NONE;
        } else {
            double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            if (yaw > 10) return propPosition.RIGHT;
            else if (yaw < -10) return propPosition.LEFT;
            else return propPosition.CENTER;
        }
    }

    private OptionalDouble getBearingToTag(int tagID) {
        OptionalDouble bearing = OptionalDouble.empty();
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && detection.id == tagID) {
                bearing = OptionalDouble.of(detection.ftcPose.bearing);
            }
        }
        return bearing;
    }

    private OptionalDouble getRangeToTag(int tagID) {
        OptionalDouble range = OptionalDouble.empty();
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && detection.id == tagID) {
                range = OptionalDouble.of(detection.ftcPose.range);
            }
        }
        return range;
    }

    private void driveForwardDistance(double distance) {
        telemetry.addData("Drive forward ", distance);
        telemetry.update();
    }

    private void rotateRobot(double rotation) {
        telemetry.addData("Rotate robot ", rotation);
        telemetry.update();
        sleep(1000);
    }

    private void deployPurplePixel() {
        telemetry.addLine("Deploy purple pixel.");
        telemetry.update();
        sleep(1000);
    }

    private void deployYellowPixel() {
        telemetry.addLine("Deploy yellow pixel.");
        telemetry.update();
        sleep(1000);
    }
}


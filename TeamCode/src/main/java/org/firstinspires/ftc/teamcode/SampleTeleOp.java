package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

//Call teleop so it shows up on the driver station
@TeleOp
public class SampleTeleOp extends LinearOpMode {
    //can have variables and define hardware objects here, anything from here to "waitForStart();" will run in initialization.

    //Change to AdafruitBNO055IMU, BNO055IMU or BHI260IMU based on what you have
    private AdafruitBNO055IMU imu;

    //variables for the automatic imu reset
    double prevImuValue = 0;
    double imuValue = 0;
    double imuDifference = 0;
    double imuWrap = 0;

    //start of opmode, inside this function will be your main while loop and initialize all hardware objects
    @Override
    public void runOpMode() {

        //IMU intialization, you will need to change this to  match your imu
            imu = hardwareMap.get(AdafruitBNO055IMU.class, "imu");
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            imu.initialize(parameters);

        //initialize motors, you will need to change these parameters to match your motor setup and names.
            Motor leftFront = new Motor(hardwareMap, "leftFront", Motor.GoBILDA.RPM_312);
            Motor rightFront = new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_312);
            Motor leftBack = new Motor(hardwareMap, "leftBack", Motor.GoBILDA.RPM_312);
            Motor rightBack = new Motor(hardwareMap, "rightBack", Motor.GoBILDA.RPM_312);

        //change the braking behavior, this is mostly personal preference but I recommend leaving this unchanged.
            leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            leftBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            rightBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        //initialize our mecanum drive from ftclib
            com.arcrobotics.ftclib.drivebase.MecanumDrive drive = new MecanumDrive(
                    leftFront,
                    rightFront,
                    leftBack,
                    rightBack
            );

        //initialize controllers
            GamepadEx driver1 = new GamepadEx(gamepad1);
            GamepadEx driver2 = new GamepadEx(gamepad2);


        //initialize servo object
            ServoEx servo1 = new SimpleServo(
                    hardwareMap, "servo1", 0, 355, AngleUnit.DEGREES
            );


        //example of how to define a motor object for basic encoder-based position control
            //initialize lift motor object, this does require an encoder
            Motor lift = new Motor(hardwareMap, "lift", Motor.GoBILDA.RPM_30);
            //set to true if this motor needs to be reversed
            lift.setInverted(true);
            //set runmode to position control
            lift.setRunMode(Motor.RunMode.PositionControl);

            //example of how to define a motor for raw power based control.
                //initialize intake motor object, does not require an encoder
                    Motor intake = new Motor(hardwareMap, "intake", Motor.GoBILDA.RPM_435);
                //set motor direction
                    lift.setInverted(true);
                //set runmode to raw power
                    lift.setRunMode(Motor.RunMode.RawPower);
//hello

        //wait for the driver station to start
            waitForStart();

    //primary while loop to call your various functions during driver control from
        while(opModeIsActive() && !isStopRequested()) {

            //read controller buttons
                driver1.readButtons();
                driver2.readButtons();

            //example of basic servo control
                if (driver1.getButton(GamepadKeys.Button.A)) {
                    servo1.setPosition(0);
                    }
                else if (driver1.getButton(GamepadKeys.Button.B)) {
                    servo1.setPosition(1);
                    }

            //example of basic position based motor control
                //can use .wasJustPressed because the lift target position only needs to change once.
                    if(driver1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                        lift.setTargetPosition(1000);
                        }
                    else if (driver1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                        lift.setTargetPosition(0);
                        }

                /*if it's not at target position spin the motor, because it's in position based control
                the direction and magnitude will automatically change themselves*/
                    if (!lift.atTargetPosition()) {
                        lift.set(0.8);
                        }

            //example of basic raw power based motor control

                //use an if else statement first so that theres a tolerance before it starts moving the motor
                    if(driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1) {
                        //set the power of the motor directly to how much the trigger is pressed down
                        intake.set(driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
                        }
                    else if(driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1) {
                        //make the right trigger spin the motor in the opposite direction
                        intake.set(-driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
                        }


            /*Code to automatically reset the imu to the last known setting if it gets reset because of static.
            I highly recommend leaving this unchanged unless you know what you're doing*/

            //get imu value as a variable
                imuValue = imu.getAngularOrientation().firstAngle + imuDifference;
            //find difference between current and previous values, wrapped around 360
                imuWrap = prevImuValue - imuValue;
                imuWrap -= 360 * Math.floor(0.5 + imuWrap / 360);
            //if difference between previous
                if(Math.abs(imuWrap) > 30) {
                    leftFront.set(0); rightFront.set(0); leftBack.set(0); rightBack.set(0);
                    //give driver feedback in the form of a short rumble if the imu has to reset because of static
                    gamepad1.rumble(500);
                    imu.initialize();
                    imuDifference = imuWrap;
                    imuValue = imu.getAngularOrientation().firstAngle + imuDifference;
                }
                prevImuValue = imuValue;


            //Code to manually reset the imu, you can change this to whatever button you want
                if (driver1.getButton(GamepadKeys.Button.DPAD_DOWN)) {
                    imu.initialize();
                    imuValue = 0;
                    prevImuValue = 0;
                    imuDifference = 0;
                }

            /*call our mecanum drive function from ftclib using field centric control,
            if you want robotcentric, change "Field" to "Robot" and remove the imuValue variable,
            if you want exponential drive turned off, change the last variable to false*/
                drive.driveFieldCentric(
                        driver1.getLeftX(),
                        driver1.getLeftY(),
                        driver1.getRightX(),
                        imuValue,
                        true
                    );
        }
    }
}

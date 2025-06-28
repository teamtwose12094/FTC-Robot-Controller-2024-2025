package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Basic: Omni Linear OpMode OLD", group="Linear OpMode")
public class BasicOmniOpMode_LinearOLD extends LinearOpMode {

    // Declare motors and servos
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;

    private DcMotor arm = null;

    private DcMotor slide = null;

    private Servo gripper = null;

    private Servo rotator = null;

    private Servo bucket = null;

    Config config = new Config();

    //Drive Motor Paramaters
    double driveSpeedMultiplier = config.driveSpeedMultiplier;
    double pivotSpeedMultiplier = config.pivotSpeedMultiplier;

    //Arm
    double armPosition = 0;
    double slidePosition = 0;


    @Override
    public void runOpMode() {

        // Hardware initialized
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");
        arm = hardwareMap.get(DcMotor.class, "arm");
        slide = hardwareMap.get(DcMotor.class, "slide");
        gripper = hardwareMap.get(Servo.class, "gripper");
        rotator = hardwareMap.get(Servo.class, "rotator");
        bucket = hardwareMap.get(Servo.class, "bucket");

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.REVERSE);
        slide.setDirection(DcMotor.Direction.FORWARD);
        gripper.setDirection(Servo.Direction.FORWARD);
        rotator.setDirection(Servo.Direction.REVERSE);
        bucket.setDirection(Servo.Direction.FORWARD);



        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Slow mode
            if (gamepad1.right_trigger > 0.1){
                driveSpeedMultiplier = config.slowDriveSpeedMultiplier;
                pivotSpeedMultiplier = config.slowPivotSpeedMultiplier;
            } else {
                driveSpeedMultiplier = config.driveSpeedMultiplier;
                pivotSpeedMultiplier = config.pivotSpeedMultiplier;

            }
            double max = 0.0;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y * driveSpeedMultiplier;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x * driveSpeedMultiplier;
            double yaw = gamepad1.right_stick_x * pivotSpeedMultiplier;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.

            // Slow mode

            double frontLeftPower = (axial + lateral + yaw);
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower = axial - lateral + yaw;
            double backRightPower = axial + lateral - yaw;


            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }


            // Send calculated power to wheels
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);


            // Arm controls (Gamepad 2)
            if (gamepad2.dpad_up) {
                armPosition += config.armStep;
                rotator.setPosition(0.4);
            } else if (gamepad2.dpad_down) {
                armPosition -= config.armStep;
                rotator.setPosition(0);
            }

            arm.setTargetPosition((int) armPosition);
            arm.setPower(Math.min(1, Math.abs(armPosition - arm.getCurrentPosition()) / 100.0)); // We can adjust the power maybe or set back to 1

            // Rotator separate controls

            if (gamepad2.dpad_left) {
                rotator.setPosition(1);
            } else if (gamepad2.dpad_right) {
                rotator.setPosition(0);
            }

            // Slide controls (Gamepad 2)
            if (gamepad2.left_trigger > 0.1 && slidePosition < config.slideMaximumPosition) {
                slidePosition += config.slideStep;
            } else if (gamepad2.right_trigger > 0.1 && slidePosition > config.slideMinimumPosition) {
                slidePosition -= config.slideStep;
            }

            // this one is for the top bucket, comment if error
            if (gamepad2.left_stick_button) {
                slidePosition = config.slideMaximumPositionHigh;
            }

            if (gamepad2.left_bumper) {
                slidePosition = config.slideMinimumSpecimen;
            }

            if (gamepad2.right_bumper) {
                slidePosition = config.slideMaximumSpecimen;
            }



            slide.setTargetPosition((int) slidePosition);
            slide.setPower(1); // We can adjust the power maybe





            // Motor set run to position out of loop

            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Gripper controls

            if (gamepad2.circle) {
                gripper.setPosition(1);
            }
            if (gamepad2.cross) {
                gripper.setPosition(0);
            }

            // Bucket controls

            if (gamepad1.left_bumper) {
                bucket.setPosition(0);
            }

            if (gamepad1.right_bumper) {
                bucket.setPosition(1);
            }


            // telemetry

            telemetry.addData("wheel", frontLeftDrive.getCurrentPosition());

            telemetry.addData("Rotator", rotator.getPosition());
            telemetry.addData("Gripper", gripper.getPosition());
            telemetry.addData("Bucket", bucket.getPosition());
            telemetry.addData("Slider", slidePosition);
            telemetry.addData("Arm", armPosition);

            telemetry.addData("gamepad1 controls", "");
            telemetry.addData("Right trigger", "Slow mode");
            telemetry.addData("Left stick up", "move forwards");
            telemetry.addData("Left stick back", "move backwards");
            telemetry.addData("right stick left", "turn left");
            telemetry.addData("right stick right", "turn right");
            telemetry.addData("left stick left", "strafe left");
            telemetry.addData("left stick right", "strafe right");
            telemetry.addData("bucket up", "left_bumper");
            telemetry.addData("bucket down", "right_bumper");
            telemetry.addData("gamepad2 controls", "");
            telemetry.addData("cross", "gripper clamp");
            telemetry.addData("circle", "gripper let go");
            telemetry.addData("d pad up", "arm up");
            telemetry.addData("d pad down", "arm down");
            telemetry.addData("d pad left", "rotator right");
            telemetry.addData("d pad right", "rotator left");
            telemetry.addData("left trigger", "slide to low basket");
            telemetry.addData("Left joystick button", "slide to high basket");
            telemetry.addData("right trigger", "slide to 0");
            telemetry.addData("Left trigger", "slide to specimen");
            telemetry.addData("right trigger", "slide high bar");

            telemetry.update();

        }
    }
}
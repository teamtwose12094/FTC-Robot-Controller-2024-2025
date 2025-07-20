package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

//import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
@Autonomous(name = "AutoYellow", group = "Autonomous")
public class AutoYellow extends LinearOpMode{

    //Declare motors and servos
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor arm = null;
    private DcMotor slide = null;
    private Servo gripper = null;
    private Servo rotator = null;
    private Servo bucket = null;
    private ElapsedTime runtime = new ElapsedTime();



@Override
    public void runOpMode() {

        telemetry.addData("Status", "Initializing Hardware..."); // Add telemetry to see progress
        telemetry.update();

        //Initialize Hardware

        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");
        arm = hardwareMap.get(DcMotor.class, "arm");
        slide = hardwareMap.get(DcMotor.class, "slide");
        gripper = hardwareMap.get(Servo.class, "gripper");
        rotator = hardwareMap.get(Servo.class, "rotator");
        bucket = hardwareMap.get(Servo.class, "bucket");

        //Set Modes
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Set Directions
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.REVERSE);
        slide.setDirection(DcMotor.Direction.FORWARD);
        gripper.setDirection(Servo.Direction.FORWARD);
        rotator.setDirection(Servo.Direction.REVERSE);
        bucket.setDirection(Servo.Direction.FORWARD);

        telemetry.addData("Status", "Hardware Initialized. Waiting for Start.");
        telemetry.update();

        waitForStart();

        runtime.reset(); // Resets the timer once the OpMode start

        telemetry.addData("Path", "Started Autonomous");
        telemetry.update();

        // Example: Drive forward for 2 seconds
        frontLeftDrive.setPower(0.5);
        backLeftDrive.setPower(0.5);
        frontRightDrive.setPower(0.5);
        backRightDrive.setPower(0.5);
        while (opModeIsActive() && runtime.seconds() < 2.0) {
            telemetry.addData("Path", "Driving Forward: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
            idle();
        }



}
}


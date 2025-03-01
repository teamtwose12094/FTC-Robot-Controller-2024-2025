package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "Yellow", group = "Autonomous")
public class Yellow extends LinearOpMode {
    public class Slide {
        private DcMotor slide;
        public Slide(HardwareMap hardwareMap) {


            slide = hardwareMap.get(DcMotor.class, "slide");
            slide.setDirection(DcMotor.Direction.FORWARD);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        public class slideToWall implements Action{
            @Override

            public boolean run(@NonNull TelemetryPacket packet) {
                slide.setTargetPosition(100);

                return false;
            }

        }

        public Action slideToWall() {
            return new slideToWall();
        }
        public class slideToHighBar implements Action{
            @Override

            public boolean run(@NonNull TelemetryPacket packet) {
                slide.setTargetPosition(1400);

                return false;
            }


        }

        public Action slideToHighBar() {
            return new slideToHighBar();
        }

        public class slideToHighBasket implements Action{
            @Override

            public boolean run(@NonNull TelemetryPacket packet) {
                slide.setTargetPosition(2500);

                return false;
            }


        }

        public Action slideToHighBasket() {
            return new slideToHighBasket();
        }

        public class slideToZero implements Action {
            @Override

            public boolean run(@NonNull TelemetryPacket packet) {
                slide.setTargetPosition(0);

                return false;
            }
        }
        public Action slideToZero() {
            return new slideToZero();
        }
    }

    public class Arm {
        private DcMotor arm;
        public Arm(HardwareMap hardwareMap) {


            arm = hardwareMap.get(DcMotor.class, ("arm"));
            arm.setDirection(DcMotor.Direction.REVERSE);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }



        public class armDown implements Action{
            @Override

            public boolean run(@NonNull TelemetryPacket packet) {
                arm.setTargetPosition(0);

                return false;
            }


        }

        public Action armDown() {
            return new armDown();
        }



        public class armUp implements Action{
            @Override

            public boolean run(@NonNull TelemetryPacket packet) {
                arm.setTargetPosition(1000);

                return false;
            }


        }
        public Action armUp() {
            return new armUp();
        }

    }

    public class Gripper {
        private Servo gripper;

        public Gripper(HardwareMap hardwareMap) {


            gripper = hardwareMap.get(Servo.class, ("gripper"));
            gripper.setDirection(Servo.Direction.REVERSE);
        }

        public class gripperOpen implements Action{
            @Override

            public boolean run(@NonNull TelemetryPacket packet) {
                gripper.setPosition(1);

                return false;
            }


        }

        public Action gripperOpen() {
            return new gripperOpen();
        }

        public class gripperClose implements Action{
            @Override

            public boolean run(@NonNull TelemetryPacket packet) {
                gripper.setPosition(0);

                return false;
            }


        }

        public Action gripperClose() {
            return new gripperClose();
        }

    }

    public class Rotator {
        private Servo rotator;

        public Rotator(HardwareMap hardwareMap) {


            rotator = hardwareMap.get(Servo.class, ("rotator"));
            rotator.setDirection(Servo.Direction.REVERSE);
        }


        public class rotatorUp implements Action{
            @Override

            public boolean run(@NonNull TelemetryPacket packet) {
                rotator.setPosition(0.6);

                return false;
            }


        }

        public Action rotatorUp() {
            return new rotatorUp();
        }

        public class rotatorDown implements Action{
            @Override

            public boolean run(@NonNull TelemetryPacket packet) {
                rotator.setPosition(0);

                return false;
            }

        }

        public Action rotatorDown() {
            return new rotatorDown();
        }
    }

    public class Bucket {
        private Servo bucket;

        public Bucket(HardwareMap hardwareMap) {


            bucket = hardwareMap.get(Servo.class, ("bucket"));
            bucket.setDirection(Servo.Direction.FORWARD);
        }

        public class bucketDown implements Action{
            @Override

            public boolean run(@NonNull TelemetryPacket packet) {
                bucket.setPosition(1);

                return false;
            }

        }

        public Action bucketDown() {
            return new bucketDown();
        }

        public class bucketUp implements Action{
            @Override

            public boolean run(@NonNull TelemetryPacket packet) {
                bucket.setPosition(0.5);

                return false;
            }

        }

        public Action bucketUp() {
            return new bucketUp();
        }
    }







    @Override
    public void runOpMode() {
        // Initialize the starting pose
        Pose2d beginPose = new Pose2d(0, 0, 0);

        // Initialize the drive system
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        // Initialize the hardware
        Slide slide = new Slide(hardwareMap);

        Arm arm = new Arm(hardwareMap);

        Rotator rotator = new Rotator(hardwareMap);

        Gripper gripper = new Gripper(hardwareMap);

        Bucket bucket = new Bucket(hardwareMap);

        // Wait for the start button to be pressed
        waitForStart();

        if (isStopRequested()) return;

        // Run the autonomous sequence
        Actions.runBlocking(
                new SequentialAction(
                        slide.slideToHighBasket(), // Move slide to high basket
                        drive.actionBuilder(beginPose)
                                .lineToX(30) // Move to X = 30
                                .build(),
                        slide.slideToWall(), // Move slide to wall
                        drive.actionBuilder(new Pose2d(30, 0, 0))
                                .turn(Math.toRadians(90)) // Turn 90 degrees
                                .lineToY(30) // Move to Y = 30
                                .turn(Math.toRadians(90)) // Turn 90 degrees
                                .lineToX(0) // Move back to X = 0
                                .turn(Math.toRadians(90)) // Turn 90 degrees
                                .lineToY(0) // Move back to Y = 0
                                .turn(Math.toRadians(90)) // Turn 90 degrees
                                .build()
                )
        );
    }
}

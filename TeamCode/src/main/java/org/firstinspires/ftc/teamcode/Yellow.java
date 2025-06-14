package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.TimeUnit;

@Config
@Autonomous(name = "Yellow", group = "Autonomous")
public class Yellow extends LinearOpMode {
    public class Slide {
        private DcMotor slide;
        public Slide(HardwareMap hardwareMap) {


            slide = hardwareMap.get(DcMotor.class, "slide");
            slide.setDirection(DcMotor.Direction.FORWARD);

        }

        public class slideToWall implements Action{
            //@Override

            public boolean run(@NonNull TelemetryPacket packet) {
                slide.setTargetPosition(100);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(0.5);

                return false;
            }

        }

        public Action slideToWall() {
            return new slideToWall();
        }
        public class slideToHighBar implements Action{
            //@Override

            public boolean run(@NonNull TelemetryPacket packet) {
                slide.setTargetPosition(1400);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(0.5);

                return false;
            }


        }

        public Action slideToHighBar() {
            return new slideToHighBar();
        }

        public class slideToHighBasket implements Action{
            //@Override

            public boolean run(@NonNull TelemetryPacket packet) {
                slide.setTargetPosition(5000);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(0.5);

                return false;
            }


        }

        public Action slideToHighBasket() {
            return new slideToHighBasket();
        }

        public class slideToZero implements Action {
            //@Override

            public boolean run(@NonNull TelemetryPacket packet) {
                slide.setTargetPosition(0);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(0.5);

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
        }



        public class armDown implements Action{
            //@Override

            public boolean run(@NonNull TelemetryPacket packet) {
                arm.setTargetPosition(-295);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.5);

                return false;
            }


        }

        public Action armDown() {
            return new armDown();
        }



        public class armUp implements Action{
            //@Override

            public boolean run(@NonNull TelemetryPacket packet) {
                arm.setTargetPosition(470);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.5);

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
            //@Override

            public boolean run(@NonNull TelemetryPacket packet) {
                gripper.setPosition(1);

                return false;
            }


        }

        public Action gripperOpen() {
            return new gripperOpen();
        }

        public class gripperClose implements Action{
            //@Override

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
            //@Override

            public boolean run(@NonNull TelemetryPacket packet) {
                rotator.setPosition(1);

                return false;
            }


        }

        public Action rotatorUp() {
            return new rotatorUp();
        }

        public class rotatorDown implements Action{
           // @Override

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
            //@Override

            public boolean run(@NonNull TelemetryPacket packet) {
                bucket.setPosition(1);

                return false;
            }

        }

        public Action bucketDown() {
            return new bucketDown();
        }

        public class bucketUp implements Action{
            //@Override

            public boolean run(@NonNull TelemetryPacket packet) {
                bucket.setPosition(0);

                return false;
            }

        }

        public Action bucketUp() {
            return new bucketUp();
        }
    }







    //@Override
    public void runOpMode() {
        // Initialize the starting pose
        Pose2d beginPose = new Pose2d(7.5, -62.5, 0);

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
                        bucket.bucketUp(),
                        drive.actionBuilder(beginPose) // Move to basket
                                .strafeTo(new Vector2d(7.5, -55))
                                .strafeTo(new Vector2d(-50, -55))
                                .turn(Math.toRadians(45))
                                .strafeTo(new Vector2d(-54, -54))
                                .build(),
                        slide.slideToHighBasket(), // Move slide to high basket
                        //new SleepAction(3),
                        bucket.bucketDown(),
                         //new SleepAction(2),
                        bucket.bucketUp(),
                        slide.slideToZero(),
                        //new SleepAction(3),
                        drive.actionBuilder(new Pose2d(-54, -54, Math.toRadians(45)))
                            .turn(Math.toRadians(45))
                            .strafeTo(new Vector2d(-45, -45))
                            .build(),
                        gripper.gripperOpen(),
                        rotator.rotatorDown(),
                        arm.armDown(),
                        gripper.gripperClose(),
                        arm.armUp(),
                        rotator.rotatorUp(),
                        gripper.gripperOpen(),
                        drive.actionBuilder(new Pose2d(-45, -45, Math.toRadians(90)))
                                .turn(Math.toRadians(45))
                                .strafeTo(new Vector2d(-54, -54))
                                .build(),
                        slide.slideToHighBasket(),
                        bucket.bucketDown(),
                        bucket.bucketUp(),
                        slide.slideToZero(),

                        drive.actionBuilder(new Pose2d(-54, -54, Math.toRadians(45)))
                                .turn(Math.toRadians(45))
                                .strafeTo(new Vector2d(-55, -45))
                                .build(),
                        gripper.gripperOpen(),
                        rotator.rotatorDown(),
                        arm.armDown(),
                        gripper.gripperClose(),
                        arm.armUp(),
                        rotator.rotatorUp(),
                        gripper.gripperOpen(),
                        drive.actionBuilder(new Pose2d(-55, -45, Math.toRadians(90)))
                                .turn(Math.toRadians(45))
                                .strafeTo(new Vector2d(-54, -54))
                                .build(),
                        slide.slideToHighBasket(),
                        bucket.bucketDown(),
                        bucket.bucketUp(),
                        slide.slideToZero(),

                        drive.actionBuilder(new Pose2d(-54, -54, Math.toRadians(45)))
                                .turn(Math.toRadians(45))
                                .strafeTo(new Vector2d(-65, -45))
                                .build(),
                        gripper.gripperOpen(),
                        rotator.rotatorDown(),
                        arm.armDown(),
                        gripper.gripperClose(),
                        arm.armUp(),
                        rotator.rotatorUp(),
                        gripper.gripperOpen(),
                        drive.actionBuilder(new Pose2d(-65, -45, Math.toRadians(90)))
                                .turn(Math.toRadians(45))
                                .strafeTo(new Vector2d(-54, -54))
                                .build(),
                        slide.slideToHighBasket(),
                        bucket.bucketDown(),
                        bucket.bucketUp(),
                        slide.slideToZero()








                )
        );

    }
}

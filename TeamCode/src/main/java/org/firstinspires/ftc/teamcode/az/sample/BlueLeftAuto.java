package org.firstinspires.ftc.teamcode.az.sample;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config
@Autonomous
public class BlueLeftAuto extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();

    SpecimenTool specimenTool = null;
    Arm arm = null;
    Slides slides = null;
    DistanceSensor distanceSensor;
    private MecanumDrive drive;
    private Pose2d beginPose;


    public class specimenHang implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            AZUtil.runInParallel(new Runnable() {
                @Override
                public void run() {

                    specimenTool.specimenHang();
                    sleep(2000);

//                    specimenTool.sampleDrop();
//                    sleep(200);

                    specimenTool.reset();
                    sleep(1000);

                }
            });
            return false;
        }
    }

    public Action specimenHang() {
        return new specimenHang();
    }

    public void initAuto() {

        arm = new Arm(this);
        slides = new Slides(this);
        specimenTool = new SpecimenTool(this);
        specimenTool.reset();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        runtime.reset();
        beginPose = new Pose2d(0,0,Math.toRadians(0));
        drive = new MecanumDrive(hardwareMap, beginPose);
        telemetry.addData("current position", drive.pose);
        telemetry.update();
        waitForStart();

    }



    public void runOpMode() throws InterruptedException {
        initAuto();

        Action specimenDropPos = drive.actionBuilder(beginPose)
                .splineToConstantHeading(new Vector2d(21, 8), Math.toRadians(0))
//                .lineToYConstantHeading(8)
                .build();


        Actions.runBlocking(
                new SequentialAction(
                        specimenHang(),
                        specimenDropPos
                )
        );
//
        sleep(5000);
        telemetry.addData("current position",drive.pose);
        telemetry.update();
        sleep(10000);



    }

}

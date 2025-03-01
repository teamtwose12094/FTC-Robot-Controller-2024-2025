package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Config
@Autonomous(name = "Yellow", group = "Autonomous")


public class Yellow extends BasicOmniOpMode_Linear{
    @Override
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(0, 0, 0);

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

            if(isStopRequested()) return;

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .lineToX(30)
                            .turn(Math.toRadians(90))
                            .lineToY(30)
                            .turn(Math.toRadians(90))
                            .lineToX(0)
                            .turn(Math.toRadians(90))
                            .lineToY(0)
                            .turn(Math.toRadians(90))
                            .build());






    }
}


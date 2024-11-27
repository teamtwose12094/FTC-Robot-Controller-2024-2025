package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TestServo extends LinearOpMode {
    @Override
    public void runOpMode() {

//        Servo left = hardwareMap.get(Servo.class, "intakeServoLeft");
//        Servo right = hardwareMap.get(Servo.class, "intakeServoRight");
          Servo bucket = hardwareMap.get(Servo.class, "bucketServo");

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
//                left.setPosition(0.83);
//                right.setPosition(0.83);
                bucket.setPosition(1);
            }

            if (gamepad1.x) {
//                left.setPosition(0.5);
//                right.setPosition(0.5);
                bucket.setPosition(0.5);
            }

            if (gamepad1.b) {
//                left.setPosition(0.05);
//                right.setPosition(0.05);b
                bucket.setPosition(0);
            }


        }
    }

}

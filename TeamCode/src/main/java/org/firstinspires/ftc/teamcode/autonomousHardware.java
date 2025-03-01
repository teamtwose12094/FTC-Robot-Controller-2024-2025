package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Servo;

import androidx.annotation.NonNull;

public class autonomousHardware {

    public DcMotor slide;

    public DcMotor arm = null;

    public Servo gripper = null;

    public Servo rotator = null;

    public Servo bucket = null;


    autonomousHardware(HardwareMap hardwareMap){

        slide = hardwareMap.get(DcMotor.class, ("slide"));
        arm = hardwareMap.get(DcMotor.class, ("arm"));
        gripper = hardwareMap.get(Servo.class, ("gripper"));
        rotator = hardwareMap.get(Servo.class, ("rotator"));
        bucket = hardwareMap.get(Servo.class, ("bucket"));

        arm.setDirection(DcMotor.Direction.REVERSE);
        slide.setDirection(DcMotor.Direction.FORWARD);
        gripper.setDirection(Servo.Direction.REVERSE);
        rotator.setDirection(Servo.Direction.REVERSE);
        bucket.setDirection(Servo.Direction.FORWARD);

    }



    // ROTATOR COMMANDS

    public class rotatorUp implements Action{
        @Override

        public boolean run(@NonNull TelemetryPacket packet) {
            rotator.setPosition(0.6);

            return false;
        }


    }

    public class rotatorDown implements Action{
        @Override

        public boolean run(@NonNull TelemetryPacket packet) {
            rotator.setPosition(0);

            return false;
        }

    }
    public class bucketDown implements Action{
        @Override

        public boolean run(@NonNull TelemetryPacket packet) {
            rotator.setPosition(1);

            return false;
        }

    }

    public class bucketUp implements Action{
        @Override

        public boolean run(@NonNull TelemetryPacket packet) {
            rotator.setPosition(0.5);

            return false;
        }

    }





    }
}

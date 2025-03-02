package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(24, -62.5, 0))
                .strafeTo(new Vector2d(7.5, -55))
                .strafeTo(new Vector2d(-50, -55))
                .turn(Math.toRadians(45))
                .strafeTo(new Vector2d(-54, -53))
                // slide up
                // bucket down
                // bucket up
                // slide down
                .turn(Math.toRadians(45))
                .strafeTo(new Vector2d(-45, -45))
                // gripper open
                //rotator down
                // arm down
                //gripper close
                // arm Up
                // rotator up
                // gripper open
                .turn(Math.toRadians(-45))
                .strafeTo(new Vector2d(-54, -53))
                // slide up
                // bucket down
                // bucket up
                // slide down
                .turn(Math.toRadians(45))
                .strafeTo(new Vector2d(-55, -45))
                // gripper open
                //rotator down
                // arm down
                //gripper close
                // arm Up
                // rotator up
                // gripper open
                .turn(Math.toRadians(-45))
                .strafeTo(new Vector2d(-54, -53))
                // slide up
                // bucket down
                // bucket up
                // slide down
                .turn(Math.toRadians(45))
                .strafeTo(new Vector2d(-65, -45))
                // gripper open
                //rotator down
                // arm down
                //gripper close
                // arm Up
                // rotator up
                // gripper open
                .turn(Math.toRadians(-45))
                .strafeTo(new Vector2d(-54, -53))
                // slide up
                // bucket down
                // bucket up
                // slide down






                .build());








        myBot.setDimensions(14, 16);

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
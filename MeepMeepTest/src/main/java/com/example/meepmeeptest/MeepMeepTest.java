package com.example.meepmeeptest;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTest {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        int leftRight = 1;
        Pose2d newStartPose;
        newStartPose = new Pose2d((-6 * Params.HALF_MAT + Params.CHASSIS_LENGTH / 2),(leftRight * Params.CHASSIS_HALF_WIDTH),0.0);
        Vector2d moveForward = new Vector2d(2 * Params.HALF_MAT, 0);
        Vector2d moveToBall;

        Pose2d shootPos = new Pose2d(Params.HALF_MAT, Params.HALF_MAT, Math.toRadians(45));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(newStartPose)
                        //.afterTime(0.3, new armToPickUpPos()) // lower arm during spline moving
                        .strafeToLinearHeading(new Vector2d(shootPos.getX(), shootPos.getY()), shootPos.getHeading())
                        .strafeToLinearHeading(new Vector2d(-3 * Params.HALF_MAT,3 * Params.HALF_MAT),Math.toRadians(90))
                        .strafeTo(new Vector2d(-3 * Params.HALF_MAT, 3.75 * Params.HALF_MAT))
                        .splineToLinearHeading(shootPos, Math.toRadians(0))
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
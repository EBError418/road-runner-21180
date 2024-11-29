package com.example.meepmeeptest;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTest {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d newStartPose = new Pose2d((-6 * Params.HALF_MAT + Params.CHASSIS_LENGTH / 2),(- Params.CHASSIS_HALF_WIDTH),Math.toRadians(179.9998));

        Vector2d firstHighChamberPos = new Vector2d(-3.5 * Params.HALF_MAT, newStartPose.getY());

        Vector2d clearSubForSamples = new Vector2d(- 2.7 * Params.HALF_MAT, - 3.0 * Params.HALF_MAT);
        Vector2d behindSamplePos = new Vector2d(- 0.5 * Params.HALF_MAT, - 3.8 * Params.HALF_MAT);
        Vector2d obsZone = new Vector2d(- 3.3 * Params.HALF_MAT, - 3.8 * Params.HALF_MAT);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(firstHighChamberPos, Math.toRadians(179.9998)))
                        //.afterTime(0.3, new armToPickUpPos()) // lower arm during spline moving
                        .splineToConstantHeading(clearSubForSamples, Math.toRadians(0))
                        .splineToConstantHeading(behindSamplePos, Math.toRadians(-110))
                        //.strafeTo(new Vector2d(behindSamplePos.getX() - 0.5 * Params.HALF_MAT, clearSubForSamples.getY() - 0.5 * Params.HALF_MAT))
                        //.strafeTo(behindSamplePos)
                        .strafeTo(new Vector2d(obsZone.getX() - 1.2 * Params.HALF_MAT, behindSamplePos.getY())) // moving to obs zone
                        .splineToConstantHeading(new Vector2d(behindSamplePos.getX(), behindSamplePos.getY()- 0.99 * Params.HALF_MAT), Math.toRadians(-135))
                        //.strafeTo(new Vector2d(behindSamplePos.getX(), - 5 * Params.HALF_MAT))//move behind 2nd sample
                        .strafeTo(new Vector2d(obsZone.getX() - 1.2 * Params.HALF_MAT, - 5 * Params.HALF_MAT)) // moving to obs zone
                        .strafeTo(new Vector2d(behindSamplePos.getX(), - 5.1 * Params.HALF_MAT))//move back
                        .strafeTo(new Vector2d(behindSamplePos.getX(), - 6 * Params.HALF_MAT + Params.CHASSIS_HALF_WIDTH))//move behind 3rd sample
                        .strafeTo(new Vector2d(obsZone.getX() - 1.2 * Params.HALF_MAT, - 6 * Params.HALF_MAT + Params.CHASSIS_HALF_WIDTH)) // moving to obs zone
                        //.afterTime(0.01, new intakeAct(0,0,intake.FINGER_OPEN)) // drop off first sample after 4 inch strafe
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
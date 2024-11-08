/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * PID controller and IMU codes are copied from
 * https://stemrobotics.cs.pdx.edu/node/7268%3Froot=4196.html
 */

package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

/**
 * Hardware config:
 *      imu on control Hub:
 *          "imu"
 *
 *      Four drive motors:
 *          "FrontLeft"
 *          "BackLeft"
 *          "BackRight"
 *          "FrontRight"
 *
 *      Servo motors:
 *          "FingerServo"
 *          "WristServo"
 *          "SwitchServo"
 *
 *      One cameras:
 *          "Webcam 1"
 */

@Autonomous(name="Right side auto with hanging", group="Concept")
//@Disabled
public class AutoRightHanging extends LinearOpMode {
    /**
     * Robot Start location: "1" - right side; "-1" - left side.
     */
    public int leftOrRight = 1;

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private intakeUnit intake;
    private MecanumDrive drive;

    Pose2d newStartPose;

    /**
     * Set robot starting location on the field:
     * 1 for Red Front, 2 for Red back, 3 for Blue Front, and 4 for Blue back
     */
    public void setRobotLocation() {
        leftOrRight = 1;
    }

    /**
     * Set robot starting position, and blueOrRed, frontOrBack variables:
     * @param leftRight : the value of robot location in the field.
     *                      -1 for left, -1 for left,
     */
    private void setStartPoses(int leftRight) {
        // road runner variables
        newStartPose = new Pose2d((-6 * Params.HALF_MAT + Params.CHASSIS_LENGTH / 2),(-leftRight * Params.CHASSIS_HALF_WIDTH),0.0);
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        Logging.log("Status - Initialized");

        setRobotLocation();

        setStartPoses(leftOrRight);

        // use slow mode if starting from front
        // updateProfileAccel(true);

        // init drive with road runner
        drive = new MecanumDrive(hardwareMap, newStartPose);
        Params.startPose = newStartPose; // init storage pose.

        intake = new intakeUnit(hardwareMap, "Arm", "Wrist", "Finger");
        intake.resetArmEncoder();

        intake.setFingerPosition(intake.FINGER_CLOSE);

        while (!isStarted()) {
            sleep(10);
            telemetry.addData( "FTC 2024 - ", "Wait for starting ");

            telemetry.addData("Arm", "position = %d", intake.getArmPosition());

            telemetry.update();
        }

        // bulk reading setting - auto refresh mode
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        waitForStart();

        Logging.log("Before start wrist pos: %2f", intake.getWristPosition());
        Logging.log("Before start finger pos: %2f", intake.getFingerPosition());
        Logging.log("Before start arm pos: %s", intake.getArmPosition());

        runtime.reset();

        // run until the end of the match (driver presses STOP)
        if (opModeIsActive()) {
            autonomousCore();
        }
    }

    private void autonomousCore() {

        autoCore();
    }

    private void autoCore() {
        Logging.log("Status - Start auto core");
        Logging.log("Auto start wrist pos: %2f", intake.getWristPosition());

        //new stuff for 2024-2025 season
        //hang specimen
        //Vector2d hangSpecimen = new Vector2d(- 3.5 * Params.HALF_MAT, 0);
        Vector2d armFlip = new Vector2d(-4.4 * Params.HALF_MAT, newStartPose.position.y);
        Vector2d retractArm = new Vector2d(- 4.9 * Params.HALF_MAT, armFlip.y);

        //grab
        Vector2d changeHeadingForPickup = new Vector2d(- 3.5 * Params.HALF_MAT, - leftOrRight * 1.85 * Params.HALF_MAT);
        Vector2d driveForwardToPickup = new Vector2d(- 3.5 * Params.HALF_MAT, - leftOrRight * 2.7 * Params.HALF_MAT);
        //Vector2d placeSample = new Vector2d(- 4.6 * Params.HALF_MAT, 4.5 * Params.HALF_MAT);
        Vector2d obsZone = new Vector2d(- 3.3 * Params.HALF_MAT, - 3.8 * Params.HALF_MAT);
        Vector2d hangSpecimenPos = new Vector2d(armFlip.x - 0.06 * Params.HALF_MAT, 0);
        Pose2d pickUpSpecimenPos = new Pose2d(- 4.06 * Params.HALF_MAT, - 6 * Params.HALF_MAT + Params.CHASSIS_HALF_WIDTH, Math.toRadians(179.9998));
        Pose2d specimenLineUpPos = new Pose2d(pickUpSpecimenPos.position.x + 0.5 * Params.HALF_MAT, pickUpSpecimenPos.position.y, Math.toRadians(179.9998));
        Vector2d splineThirdSample = new Vector2d(- 2.8 * Params.HALF_MAT, - leftOrRight * 3 * Params.HALF_MAT);

        //ascent level 1
        Vector2d parkStepOne = new Vector2d(- 4 * Params.HALF_MAT,  4 * Params.HALF_MAT);
        Vector2d parkStepTwo = new Vector2d(parkStepOne.x + 3 * Params.HALF_MAT, parkStepOne.y);
        Vector2d parkStepThree = new Vector2d(parkStepTwo.x, 1.8 * Params.HALF_MAT);
        if (leftOrRight == 1) { // right side auto
            //Go to position for arm flip and hang on high chamber
            Logging.log("X position = %2f, Y position = %2f, Heading = %2f", drive.pose.position.x, drive.pose.position.y, Math.toDegrees(drive.pose.heading.log()));
            Actions.runBlocking(
                    drive.actionBuilder(newStartPose)
                            .afterTime(0.5, new armFlipToHangAct())
                            .strafeTo(armFlip)
                            .build()
            );
            Logging.log("After arm flip pos wrist pos: %2f", intake.getWristPosition());
            Logging.log("X position = %2f, Y position = %2f, Heading = %2f", drive.pose.position.x, drive.pose.position.y, Math.toDegrees(drive.pose.heading.log()));
            sleep(900); // TODO : optimize sleep time

            updateProfileAccel(true);
            //release specimen and raise arm to clear high chamber
            intake.setFingerPosition(intake.FINGER_OPEN);
            intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER - 800);
            intake.setWristPosition(intake.WRIST_POS_HIGH_CHAMBER + 0.25);
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .afterTime(0.75, new armToPickUpPos())
                            .splineToLinearHeading(new Pose2d(changeHeadingForPickup, Math.toRadians(297)), Math.toRadians(-63))
                            .strafeTo(new Vector2d(driveForwardToPickup.x + 0.45 * Params.HALF_MAT, driveForwardToPickup.y + 0.2 * Params.HALF_MAT) )
                            .build()
            );

            //Go to pick up red sample
            updateProfileAccel(false);
            intake.setFingerPosition(intake.FINGER_CLOSE);
            sleep(100);
            intake.setArmPosition(intake.ARM_POS_OBS_ZONE);

            //place sample in observation zone
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .afterTime(0.4, new armToObsZoneAct())
                            .splineToLinearHeading(new Pose2d(obsZone, Math.toRadians(-160)), Math.toRadians(-135))
                            .build()
            );
            //drop off sample in observation zone
            intake.setFingerPosition(intake.FINGER_OPEN);

            //pick up second sample
            updateProfileAccel(true);
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .afterTime(0.4, new armToPickUpPos())
                            .strafeToLinearHeading(new Vector2d(changeHeadingForPickup.x + 0.3 * Params.HALF_MAT, changeHeadingForPickup.y - 0.85 * Params.HALF_MAT), Math.toRadians(-65))
                            .afterTime(0.35, new fingerCloseEnRouteAct())
                            .strafeTo(new Vector2d(driveForwardToPickup.x + 0.45 * Params.HALF_MAT, driveForwardToPickup.y - 0.8 * Params.HALF_MAT))
                            .afterTime(0.05, new armToObsZoneAct())
                            .strafeToLinearHeading(obsZone, Math.toRadians(-160)) // TODO : try updating heading to -160
                            .afterTime(0.2, new fingerOpenEnRouteAct())
                            .afterTime(0.05, new armToPickupSpecimen())
                            .strafeToLinearHeading(specimenLineUpPos.position, pickUpSpecimenPos.heading)
                            .strafeToConstantHeading(pickUpSpecimenPos.position)
                            .build()
            );

            updateProfileAccel(false);

            //put specimen on high chamber
            intake.setWristPosition(intake.WRIST_POS_GRAB_SPECIMEN - 0.038);
            sleep(100); //200
            intake.setFingerPosition(intake.FINGER_CLOSE);
            sleep(100); // 200
            intake.setArmPosition(intake.ARM_POS_BACK);
            sleep(200);
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            //.turnTo(Math.toRadians(-179.99)) // TODO : check if we need this fine heading adjust
                            //.afterTime(0.35, new pickUpSpecimenAct()) // TODO : move this act above this runBlocking so we can remove waiting time
                            //.afterTime(0.45, new armToBackAct())
                            //.waitSeconds(0.55) // TODO : possibly able to be removed
                            //.afterTime(0.7, new armFlipToHangAct())
                            .strafeToLinearHeading(hangSpecimenPos, 0)
                            .build()
            );
            intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER);
            intake .setWristPosition(intake.WRIST_POS_HIGH_CHAMBER);
            sleep(1100);
            intake.setFingerPosition(intake.FINGER_OPEN);

            /* start for second specimen */
            //back to observation zone for next specimen
            intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER - 850);
            intake.setWristPosition(intake.WRIST_POS_HIGH_CHAMBER + 0.2);
            sleep(300); //arm runs into hanged specimen
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .setReversed(true)
                            .afterTime(0.7, new armToPickUpPos())
                            .splineToLinearHeading(new Pose2d(splineThirdSample, Math.toRadians(-90)), Math.toRadians(-63))
                            .afterTime(0.7, new fingerCloseEnRouteAct())
                            .strafeToConstantHeading(new Vector2d(splineThirdSample.x + 0.6 * Params.HALF_MAT, splineThirdSample.y - 0.8 * Params.HALF_MAT))
                            .strafeToLinearHeading(obsZone, Math.toRadians(-160)) // TODO : update heading to -160
                            .afterTime(0.2, new fingerOpenEnRouteAct())
                            .afterTime(0.3, new armToPickupSpecimen())
                            .strafeToLinearHeading(specimenLineUpPos.position, pickUpSpecimenPos.heading)
                            .strafeToConstantHeading(new Vector2d(pickUpSpecimenPos.position.x - 0.1 * Params.HALF_MAT, pickUpSpecimenPos.position.y))
                            .afterTime(0.35, new pickUpSpecimenAct())
                            .waitSeconds(0.55)
                            .build()
            );
            updateProfileAccel(true);
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .afterTime(0.3, new armToBackAct())
                            .strafeToLinearHeading(new Vector2d(hangSpecimenPos.x + 0.1 * Params.HALF_MAT, hangSpecimenPos.y + Params.CHASSIS_HALF_WIDTH), 0)
                            .afterTime(0.2, new armFlipToHangAct())
                            .afterTime(1.35, new fingerOpenEnRouteAct())
                            .waitSeconds(1.45)
                            .afterTime(0.1, new armToBackAct())
                            .afterTime(0.1, new wristToBackAct())
                            .setReversed(true) // for parking
                            .splineTo(new Vector2d(-5.0 * Params.HALF_MAT,-4.0 * Params.HALF_MAT), Math.toRadians(-90))
                            .build()
            );
        }
        Logging.log("X position = %2f, Y position = %2f", drive.pose.position.x, drive.pose.position.y);
    }

    //action for arm flip to hang
    private class armFlipToHangAct implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER);
            intake .setWristPosition(intake.WRIST_POS_HIGH_CHAMBER);
            return false;
        }
    }

    //action for arm to pick up neutral sample
    private class armToPickUpPos implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setArmPosition(intake.ARM_POS_GRAB_SAMPLE);
            intake.setWristPosition(intake.WRIST_POS_GRAB_SAMPLE);
            intake.setFingerPosition(intake.FINGER_OPEN);
            return false;
        }
    }

    //action for arm to reach out for parking
    private class armToParkingAct implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setArmPosition(intake.ARM_POS_PARKING);
            intake.setWristPosition(intake.WRIST_POS_PARKING);
            return false;
        }
    }

    //action for arm to drop sample in bucket
    private class armToDropSampleAct implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setArmPosition(intake.ARM_POS_LOW_BUCKET);
            intake.setWristPosition(intake.WRIST_POS_LOW_BUCKET);
            return false;
        }
    }

    //action for arm to drop sample in observation zone
    private class armToObsZoneAct implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setArmPosition(intake.ARM_POS_OBS_ZONE);
            intake.setWristPosition(intake.WRIST_POS_OBS_ZONE);
            return false;
        }
    }

    private class fingerOpenEnRouteAct implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setFingerPosition(intake.FINGER_OPEN);
            return false;
        }
    }
    private class fingerCloseEnRouteAct implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setFingerPosition(intake.FINGER_CLOSE);
            return false;
        }
    }

    //action for arm to pick up specimen
    private class armToPickupSpecimen implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setArmPosition(intake.ARM_POS_GRAB_SPECIMEN);
            intake.setWristPosition(intake.WRIST_POS_GRAB_SPECIMEN);
            return false;
        }
    }

    private class pickUpSpecimenAct implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setWristPosition(intake.WRIST_POS_GRAB_SPECIMEN - 0.038);
            sleep(250);
            intake.setFingerPosition(intake.FINGER_CLOSE);
            sleep(250);
            intake.setArmPosition(intake.ARM_POS_BEFORE_HANG);
            return false;
        }
    }

    private class armToBackAct implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setArmPosition(intake.ARM_POS_BACK);
            intake.setWristPosition(intake.WRIST_POS_HIGH_CHAMBER);
            return false;
        }
    }
    private class wristToBackAct implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setWristPosition(intake.WRIST_POS_HIGH_CHAMBER + 0.5);
            return false;
        }
    }

    private void updateProfileAccel(boolean slowMode) {
        if (slowMode) {
            MecanumDrive.PARAMS.minProfileAccel = -25;
            MecanumDrive.PARAMS.maxProfileAccel = 40;
        } else {
            MecanumDrive.PARAMS.minProfileAccel = -30;
            MecanumDrive.PARAMS.maxProfileAccel = 50;
        }
    }
}
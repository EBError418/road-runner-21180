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
    public int leftOrRight = -1;

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private intakeUnit intake;

    //private SlidersWith2Motors slider;
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
        updateProfileAccel(true);

        // init drive with road runner
        drive = new MecanumDrive(hardwareMap, newStartPose);
        Params.startPose = newStartPose; // init storage pose.

        intake = new intakeUnit(hardwareMap, "Arm", "Wrist", "Finger");
        intake.resetArmEncoder();

        intake.setFingerPosition(intake.FINGER_CLOSE);

        //intake.setWristPosition(0.95);

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

        //intake.setWristPosition(0.95);

        //intake.setFingerPosition(intake.FINGER_CLOSE);

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
        //intake.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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
        Vector2d driveForwardToPickup = new Vector2d(- 3.5 * Params.HALF_MAT, - leftOrRight * 2.55 * Params.HALF_MAT);
        //Vector2d placeSample = new Vector2d(- 4.6 * Params.HALF_MAT, 4.5 * Params.HALF_MAT);
        Vector2d obsZone = new Vector2d(- 4.2 * Params.HALF_MAT, - 3.5 * Params.HALF_MAT);
        Vector2d hangSpecimenPos = new Vector2d(- 4.4 * Params.HALF_MAT, Params.CHASSIS_HALF_WIDTH);
        Pose2d pickUpSpecimenPos = new Pose2d(- 4.1 * Params.HALF_MAT, - 6 * Params.HALF_MAT + Params.CHASSIS_HALF_WIDTH + 1, Math.toRadians(179.9998));
        Pose2d specimenLineUpPos = new Pose2d(pickUpSpecimenPos.position.x + 1.3 * Params.HALF_MAT, pickUpSpecimenPos.position.y, Math.toRadians(179.9998));
        //Vector2d splineThirdSample = new Vector2d(- 2.5 * Params.HALF_MAT, - leftOrRight * 3 * Params.HALF_MAT);

        //ascent level 1
        Vector2d parkStepOne = new Vector2d(- 4 * Params.HALF_MAT,  4 * Params.HALF_MAT);
        Vector2d parkStepTwo = new Vector2d(parkStepOne.x + 3 * Params.HALF_MAT, parkStepOne.y);
        Vector2d parkStepThree = new Vector2d(parkStepTwo.x, 1.8 * Params.HALF_MAT);
        if (leftOrRight == 1) { // right side auto
            //Go to position for arm flip and hang on high chamber
            Logging.log("X position = %2f, Y position = %2f, Heading = %2f", drive.pose.position.x, drive.pose.position.y, Math.toDegrees(drive.pose.heading.log()));
            Actions.runBlocking(
                    drive.actionBuilder(newStartPose)
                            .afterTime(0.6, new armFlipToHangAct())
                            .strafeTo(armFlip)
                            .build()
            );
            Logging.log("After arm flip pos wrist pos: %2f", intake.getWristPosition());
            Logging.log("X position = %2f, Y position = %2f, Heading = %2f", drive.pose.position.x, drive.pose.position.y, Math.toDegrees(drive.pose.heading.log()));
            sleep(1600);

            //release specimen and raise arm to clear high chamber
            intake.setFingerPosition(intake.FINGER_OPEN);
            Actions.runBlocking(
                    drive.actionBuilder(newStartPose)
                            .waitSeconds(0.5)
                            .strafeToConstantHeading(retractArm)
                            .build()
            );
            sleep(100);
            intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER - 600);

            //Go to pick up red sample
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .afterTime(0.4, new armToPickUpPos())
                            .splineTo(changeHeadingForPickup, Math.toRadians(-58))
                            .turnTo(Math.toRadians(297))
                            .strafeTo(new Vector2d(driveForwardToPickup.x + 0.3 * Params.HALF_MAT, driveForwardToPickup.y + 0.2 * Params.HALF_MAT))
                            .build()
            );
            sleep(100);
            intake.setFingerPosition(intake.FINGER_CLOSE);
            sleep(200);
            intake.setArmPosition(intake.ARM_POS_OBS_ZONE);
            sleep(200);

            //place sample in observation zone
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .afterTime(0.4, new armToObsZoneAct())
                            .splineToLinearHeading(new Pose2d(obsZone, Math.toRadians(-135)), Math.toRadians(-135))
                            .build()
            );
            sleep(150);
            intake.setFingerPosition(intake.FINGER_OPEN);
            sleep(150);

            //pick up second sample
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .afterTime(0.4, new armToPickUpPos())
                            .strafeToLinearHeading(new Vector2d(changeHeadingForPickup.x + 0.3 * Params.HALF_MAT, changeHeadingForPickup.y - 0.85 * Params.HALF_MAT), Math.toRadians(-65))
                            .strafeTo(new Vector2d(driveForwardToPickup.x + 0.3 * Params.HALF_MAT, driveForwardToPickup.y - 0.7 * Params.HALF_MAT))
                            .build()
            );
            sleep(100);
            intake.setFingerPosition(intake.FINGER_CLOSE);
            sleep(200);
            intake.setArmPosition(intake.ARM_POS_OBS_ZONE);
            sleep(200);

            //place second sample in observation zone
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .afterTime(0.4, new armToObsZoneAct())
                            .splineToLinearHeading(new Pose2d(obsZone, Math.toRadians(- 135)), Math.toRadians(- 135))
                            .build()
            );
            sleep(150);
            intake.setFingerPosition(intake.FINGER_OPEN);
            sleep(150);

            //put specimen on high chamber
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(specimenLineUpPos.position, specimenLineUpPos.heading)
                            .afterTime(1.0, new armToPickupSpecimen())
                            .strafeToConstantHeading(pickUpSpecimenPos.position)
                            .afterTime(1.0, new pickUpSpecimenAct())
                            .waitSeconds(2)
                            .afterTime(0.6, new armToBackAct())
                            .strafeToLinearHeading(hangSpecimenPos, 0)
                            .build()
            );
            intake.setArmPosition(intake.ARM_POS_BACK);
            intake.setWristPosition(intake.WRIST_POS_HIGH_CHAMBER);
            sleep(1000);
            intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER);
            intake.setWristPosition(intake.WRIST_POS_HIGH_CHAMBER);
            sleep(750);
            intake.setFingerPosition(intake.FINGER_OPEN);
            sleep(500);
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(obsZone, specimenLineUpPos.heading)
                            //.afterTime(1.0, new armToPickupSpecimen())
                            //.lineToXLinearHeading(pickUpSpecimenPos.position.x, pickUpSpecimenPos.heading)
                            //.afterTime(1.0, new pickUpSpecimenAct())
                            //.strafeToLinearHeading(hangSpecimenPos, 0)
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

    private void updateProfileAccel(boolean slowMode) {
        if (slowMode) {
            MecanumDrive.PARAMS.minProfileAccel = -20;
            MecanumDrive.PARAMS.maxProfileAccel = 30;
        }
    }
}
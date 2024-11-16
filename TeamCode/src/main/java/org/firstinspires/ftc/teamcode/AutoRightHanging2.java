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

@Autonomous(name="Right v2", group="Concept")
//@Disabled
public class AutoRightHanging2 extends LinearOpMode {
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

        setStartPoses(leftOrRight);

        // init drive with road runner
        drive = new MecanumDrive(hardwareMap, newStartPose);
        Params.startPose = newStartPose; // init storage pose.

        intake = new intakeUnit(hardwareMap, "Arm", "Wrist", "Finger");
        //intake.resetArmEncoder();

        intake.setFingerPosition(intake.FINGER_CLOSE);

        while (!isStarted()) {
            sleep(10);
            telemetry.addData( "FTC 2024 - ", "Wait for starting ");

            telemetry.addData("Arm", "position = %d", intake.getArmPosition());

            telemetry.addData(" ---- ", " ---  ");

            telemetry.addData("Arm calibration - -    ", Params.armCalibrated? "Pass!" :" Failed !!!!!");

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
            if (Params.armCalibrated) {
                autonomousCore();

                Logging.log("Autonomous time - total Run Time: " + runtime);
            } else {
                telemetry.addData("Arm calibration: ---", "need to be done before starting!");
                telemetry.update();
                sleep(4000);
            }
        }

    }

    private void autonomousCore() {

        Logging.log("Status - Start auto core");
        Logging.log("Auto start wrist pos: %2f", intake.getWristPosition());

        //new stuff for 2024-2025 season
        //hang specimen
        Vector2d armFlip = new Vector2d(-4.4 * Params.HALF_MAT, newStartPose.position.y);

        //grab
        Vector2d changeHeadingForPickup = new Vector2d(- 3.5 * Params.HALF_MAT, - leftOrRight * 1.85 * Params.HALF_MAT);
        Vector2d driveForwardToPickup = new Vector2d(- 3.5 * Params.HALF_MAT, - leftOrRight * 2.7 * Params.HALF_MAT);
        Vector2d obsZone = new Vector2d(- 3.3 * Params.HALF_MAT, - 3.8 * Params.HALF_MAT);
        Vector2d hangSpecimenPos = new Vector2d(armFlip.x - 0.06 * Params.HALF_MAT, 0);
        Pose2d pickUpSpecimenPos = new Pose2d(- 5.5 * Params.HALF_MAT + Params.CHASSIS_HALF_WIDTH, - 3.96 * Params.HALF_MAT, Math.toRadians(-90.0));
        Vector2d splineThirdSample = new Vector2d(-2.2 * Params.HALF_MAT, - leftOrRight * 2.9 * Params.HALF_MAT);
        //Vector2d newSpecimenPos = new Vector2d(pickUpSpecimenPos.position.x, pickUpSpecimenPos.position.y - 0.07 * Params.HALF_MAT);
        Pose2d specimenLineUpPos = new Pose2d(pickUpSpecimenPos.position.x + 0.1 * Params.HALF_MAT, pickUpSpecimenPos.position.y + Params.HALF_MAT, Math.toRadians(-90.0));

        //ascent level 1
        Vector2d parkObz = new Vector2d(-5.0 * Params.HALF_MAT,-4.0 * Params.HALF_MAT);
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
            sleep(1100); // TODO : optimize sleep time

            updateProfileAccel(true);
            //release specimen and raise arm to clear high chamber
            intake.setFingerPosition(intake.FINGER_OPEN);
            intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER - 1200);
            intake.setWristPosition(intake.WRIST_POS_HIGH_CHAMBER + 0.15);

            sleep(300);
            //Go to pick up first sample on mat
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .afterTime(0.3, new armToPickUpPos()) // lower arm during spline moving
                            .splineToLinearHeading(new Pose2d(changeHeadingForPickup, Math.toRadians(-60)), Math.toRadians(-60))
                            .afterTime(0.35, new intakeAct(0,0, intake.FINGER_CLOSE)) // close finger during strafe
                            .strafeTo(new Vector2d(driveForwardToPickup.x + 0.5 * Params.HALF_MAT, driveForwardToPickup.y + 0.1 * Params.HALF_MAT) )
                            .afterTime(0.01, new intakeAct(intake.ARM_POS_OBS_ZONE, 0, 0)) // lift arm a little bit
                            .strafeToLinearHeading(new Vector2d(obsZone.x - 1.3 * Params.HALF_MAT, obsZone.y + Params.HALF_MAT), Math.toRadians(-120.0)) // moving to obs zone
                            .afterTime(0.01, new intakeAct(0,0,intake.FINGER_OPEN)) // drop off first sample after 4 inch strafe
                            .build()
            );

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .afterTime(0.4, new armToPickUpPos()) // pickup second sample
                            .strafeToLinearHeading(new Vector2d(changeHeadingForPickup.x + 0.4 * Params.HALF_MAT, changeHeadingForPickup.y - 0.85 * Params.HALF_MAT), Math.toRadians(-65))
                            .afterTime(0.35, new fingerCloseEnRouteAct())
                            .strafeTo(new Vector2d(driveForwardToPickup.x + 0.45 * Params.HALF_MAT, driveForwardToPickup.y - 0.75 * Params.HALF_MAT))
                            .afterTime(0.01, new intakeAct(intake.ARM_POS_OBS_ZONE, 0 /*intake.WRIST_POS_OBS_ZONE*/, 0)) // lift arm a little bit
                            .strafeToLinearHeading(new Vector2d(obsZone.x - 5.0, obsZone.y), Math.toRadians(-160))
                            .afterTime(0.01, new intakeAct(intake.ARM_POS_OBS_ZONE - 100,0,intake.FINGER_OPEN)) // drop off second sample
                            .afterTime(0.4, new armToPickUpPos()) // start for third sample
                            .splineToLinearHeading(new Pose2d(splineThirdSample, Math.toRadians(-90)), Math.toRadians(90)) // move back for picking 3rd sample
                            .afterTime(0.72, new fingerCloseEnRouteAct())
                            .afterTime(0.74, new intakeAct(intake.ARM_POS_OBS_ZONE, 0, 0)) // lift arm a little bit
                            .strafeToConstantHeading(new Vector2d(splineThirdSample.x, splineThirdSample.y - 0.99 * Params.HALF_MAT))
                            .setReversed(true) // to avoid hitting wall during strafe
                            .strafeToConstantHeading(specimenLineUpPos.position) // spline to side wall
                            .afterTime(0.01, new intakeAct(intake.ARM_POS_OBS_ZONE - 100, 0, intake.FINGER_OPEN)) // drop off 3rd sample. lift arm a little bit
                            .afterTime(0.2, new armToPickupSpecimen()) // get arm ready for first specimen pick up
                            .strafeToConstantHeading(pickUpSpecimenPos.position) // spline pick up pos
                            //.strafeToConstantHeading(pickUpSpecimenPos.position)
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

            //strafe to hanging first specimen
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            //.turnTo(Math.toRadians(-179.99)) // TODO : check if we need this fine heading adjust
                            .afterTime(1.8, new intakeAct(intake.ARM_POS_HIGH_CHAMBER, intake.WRIST_POS_HIGH_CHAMBER, 0)) // forward arm to hang first specimen
                            .strafeToLinearHeading(new Vector2d(hangSpecimenPos.x - 0.05 * Params.HALF_MAT, hangSpecimenPos.y), 0) //adjustment for error en route
                            .build()
            );
            sleep(700);
            intake.setFingerPosition(intake.FINGER_OPEN);

            /* start for second specimen */
            //back to observation zone for next specimen
            intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER - 700);
            intake.setWristPosition(intake.WRIST_POS_HIGH_CHAMBER + 0.2);
            sleep(300);
            //sleep(300); //arm runs into hanged specimen
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .setReversed(true)
                            .afterTime(1.0, new armToPickupSpecimen()) // get arm ready for second specimen pick up
                            .strafeToLinearHeading(specimenLineUpPos.position, pickUpSpecimenPos.heading) // spline to side wall for second specimen
                            .strafeToConstantHeading(pickUpSpecimenPos.position)
                            .build()
            );
            //pickup second specimen
            intake.setWristPosition(intake.WRIST_POS_GRAB_SPECIMEN - 0.038);
            sleep(100); //200
            intake.setFingerPosition(intake.FINGER_CLOSE);
            sleep(100); // 200
            intake.setArmPosition(intake.ARM_POS_BACK);
            sleep(200);

            //strafe to hanging second specimen
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            // forward arm during strafe
                            .afterTime(1.9, new intakeAct(intake.ARM_POS_HIGH_CHAMBER, intake.WRIST_POS_HIGH_CHAMBER, 0)) // forward arm to hang first specimen
                            //.strafeToLinearHeading(hangSpecimenPos, 0)
                            //.strafeToLinearHeading(new Vector2d(hangSpecimenPos.x - 0.1 * Params.HALF_MAT, hangSpecimenPos.y + Params.CHASSIS_HALF_WIDTH), 0)
                            .strafeToLinearHeading(new Vector2d(hangSpecimenPos.x - 0.05 * Params.HALF_MAT, hangSpecimenPos.y - Params.CHASSIS_HALF_WIDTH + 4.0), 0)
                            .build()
            );

            // back arm after hanging the second specimen
            sleep(900);
            intake.setFingerPosition(intake.FINGER_OPEN);
            intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER - 700);
            intake.setWristPosition(intake.WRIST_POS_HIGH_CHAMBER + 0.20);

            // back to obs zone for parking
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .setReversed(true) // for parking
                            .strafeTo(parkObz)
                            .build()
            );
            intake.setArmPosition(intake.ARM_POS_GRAB_SAMPLE);
            intake.setFingerPosition(intake.FINGER_SPECIMEN_CLOSE);
            intake.setWristPosition(intake.WRIST_POS_HANGING);

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

    private class armToBackAct implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setArmPosition(intake.ARM_POS_BACK);
            intake.setWristPosition(intake.WRIST_POS_HIGH_CHAMBER);
            return false;
        }
    }

    //action for arm, wrist, finger position control during driving
    private class intakeAct implements Action {
        public intakeAct(double armPos, double wristPos, double fingerPos) {
            arm = (int)armPos;
            wrist = wristPos;
            finger = fingerPos;
        }

        private final int arm;
        private final double wrist;
        private final double finger;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if (arm < 0) {
                intake.setArmPosition(arm);
            }

            if (wrist > 0) {
                intake.setWristPosition(wrist);
            }

            if (finger > 0) {
                intake.setFingerPosition(finger);
            }
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
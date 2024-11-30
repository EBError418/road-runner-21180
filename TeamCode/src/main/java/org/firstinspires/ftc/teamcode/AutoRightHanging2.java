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
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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
        newStartPose = new Pose2d((-6 * Params.HALF_MAT + Params.CHASSIS_LENGTH / 2),(-leftRight * Params.CHASSIS_HALF_WIDTH),Math.toRadians(180));
    }
    private DistanceSensor distSensor;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        Logging.log("Status - Initialized");

        setStartPoses(leftOrRight);

        // init drive with road runner
        drive = new MecanumDrive(hardwareMap, newStartPose);
        Params.startPose = newStartPose; // init storage pose.
        Params.currentPose = newStartPose;; // init storage pose

        intake = new intakeUnit(hardwareMap, "Arm", "Wrist", "Knuckle","Finger");
        //intake.resetArmEncoder();

        // set RunToPosition mode and set power for motors.
        intake.setWristModeRunToPosition(intake.getWristPosition());
        intake.setArmModeRunToPosition(intake.getArmPosition());


        intake.setFingerPosition(intake.FINGER_CLOSE);

        // you can use this as a regular DistanceSensor.
        distSensor = hardwareMap.get(DistanceSensor.class, "distance");

        while (!isStarted()) {
            sleep(10);
            telemetry.addData( "FTC 2024 - ", "Wait for starting ");

            telemetry.addData("deviceName", distSensor.getDeviceName() );

            telemetry.addData("range", String.format("%.01f in", distSensor.getDistance(DistanceUnit.INCH)));

            telemetry.addData("Arm", "position = %d", intake.getArmPosition());

            telemetry.addData(" ---- ", " ---  ");

            telemetry.addData("Arm calibration - -    ", Params.armCalibrated? "Pass!" :" Failed !!!!!");

            telemetry.update();
        }

        // bulk reading setting - auto refresh mode
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        waitForStart();

        Logging.log("Before start wrist pos: %s", intake.getWristPosition());
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
            Params.currentPose = drive.pose; // save current position
        }

    }

    private void autonomousCore() {

        Logging.log("Status - Start auto core");
        Logging.log("Auto start wrist pos: %s", intake.getWristPosition());

        //new stuff for 2024-2025 season
        //hang specimen
        Vector2d firstHighChamberPos = new Vector2d(-3.1 * Params.HALF_MAT, newStartPose.position.y);

        //grab
        Vector2d firstSamplePosition = new Vector2d(- 3.3 * Params.HALF_MAT, - 4.35 * Params.HALF_MAT);
        Vector2d secondSamplePosition = new Vector2d(- 3.3 * Params.HALF_MAT, firstSamplePosition.y - 10);

        Vector2d driveForwardToPickup = new Vector2d(- 3.5 * Params.HALF_MAT, - leftOrRight * 2.7 * Params.HALF_MAT);
        Vector2d obsZone = new Vector2d(- 4 * Params.HALF_MAT, - 3 * Params.HALF_MAT);
        Vector2d hangSpecimenPos = new Vector2d(firstHighChamberPos.x - 0.1 * Params.HALF_MAT, firstHighChamberPos.y);
        Pose2d pickUpSpecimenPos = new Pose2d(- 3.3 * Params.HALF_MAT, - 3.8 * Params.HALF_MAT, Math.toRadians(180));
        Vector2d splineThirdSample = new Vector2d(-2.2 * Params.HALF_MAT, - leftOrRight * 2.9 * Params.HALF_MAT);
        Pose2d specimenLineUpPos = new Pose2d(pickUpSpecimenPos.position.x +1.5, pickUpSpecimenPos.position.y + Params.HALF_MAT, Math.toRadians(-81.0));

        //TODO: fix power ramp
        //ascent level 1
        Vector2d parkObz = new Vector2d(-5.0 * Params.HALF_MAT,-4.0 * Params.HALF_MAT);

        if (leftOrRight == 1) { // right side auto
            //Go to position for arm flip and hang on high chamber
            Logging.log(" Start position: X position = %2f, Y position = %2f, Heading = %2f", drive.pose.position.x, drive.pose.position.y, Math.toDegrees(drive.pose.heading.log()));
            Actions.runBlocking(
                    drive.actionBuilder(newStartPose)
                            .afterTime(0.01, new armToReadyHangAct())
                            .strafeTo(firstHighChamberPos)
                            .build()
            );
            Logging.log("After ready hang pos wrist pos: %s", intake.getWristPosition());

            Logging.log("X position = %2f, Y position = %2f, Heading = %2f", drive.pose.position.x, drive.pose.position.y, Math.toDegrees(drive.pose.heading.log()));
            double sensorDist = distSensor.getDistance(DistanceUnit.INCH);
            double shiftDelta = sensorDist - Params.HIGH_CHAMBER_DIST;
            shiftDelta = (shiftDelta > 70)? 70 : ((shiftDelta < -70)? -70 : shiftDelta);
            Logging.log("drive pose before distance");
            Logging.log(" X position = %2f, Y position = %2f ", drive.pose.position.x, drive.pose.position.y);
            Logging.log("fist specimen sensor dist = %2f, shift delta = %2f", sensorDist, shiftDelta);

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToConstantHeading(new Vector2d(drive.pose.position.x + shiftDelta , drive.pose.position.y))
                            .build()
            );
            Logging.log("after adjust X position = %2f, Y position = %2f ", drive.pose.position.x, drive.pose.position.y);

            //hanging action
            intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER);
            sleep(200);

            //release specimen and lower arm to clear high chamber
            intake.setFingerPosition(intake.FINGER_OPEN);
            intake.setArmPosition(intake.ARM_POS_GRAB_SAMPLE_BACK + 400);
            intake.setKnucklePosition(intake.KNUCKLE_POS_HIGH_CHAMBER - 0.2);

            sleep(300);
            //Go to pick up first sample on mat
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .afterTime(1.6, new armToPickUpPos()) // lower arm during spline moving
                            .strafeToConstantHeading(firstSamplePosition)//go to first sample position
                            .turnTo(newStartPose.heading)
                            .afterTime(0.01, new fingerCloseEnRouteAct())//grab first sample
                            .afterTime(0.09, new intakeAct(intake.ARM_POS_DROP_SAMPLE, intake.WRIST_POS_NEUTRAL, intake.KNUCKLE_POS_DROP_SAMPLE, true))//go back for drop sample
                            //.afterTime(2.0, new fingerOpenEnRouteAct())//
                            .waitSeconds(0.09)
                            .strafeToConstantHeading(secondSamplePosition)//go to second sample position
                            //.turnTo(newStartPose.heading)
                            .waitSeconds(0.35)
                            .afterTime(0.01, new armToPickUpPos()) //arm to grab sample when open finger
                            .afterTime(1.65, new fingerCloseEnRouteAct())//grab second sample
                            .afterTime(1.8, new intakeAct(intake.ARM_POS_DROP_SAMPLE, intake.WRIST_POS_NEUTRAL, intake.KNUCKLE_POS_DROP_SAMPLE, true))
                            .afterTime(3.0, new fingerOpenEnRouteAct())//drop second sample
                            .waitSeconds(3.0)
                            .build()
            );

            Logging.log("after 2nd sample drop heading: %2f", Math.toDegrees(drive.pose.heading.log()));



            //loop for hanging specimen
            for (int j = 2; j <= 4; j++) {
                //pickup specimen
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .afterTime(0.1, new intakeAct(intake.ARM_POS_GRAB_SPECIMEN, intake.WRIST_POS_GRAB_SPECIMEN, intake.KNUCKLE_POS_PICKUP_SPECIMEN - 0.35, false))
                                .strafeTo(pickUpSpecimenPos.position)
                                .turnTo(newStartPose.heading)
                                .afterTime(0.1, new intakeAct(intake.ARM_POS_GRAB_SPECIMEN, intake.WRIST_POS_GRAB_SPECIMEN, intake.KNUCKLE_POS_PICKUP_SPECIMEN, false))
                                .afterTime(0.5, new fingerCloseEnRouteAct())//grab specimen
                                .afterTime(0.65, new intakeAct(intake.ARM_POS_HIGH_CHAMBER_READY, intake.WRIST_POS_HIGH_CHAMBER, intake.KNUCKLE_POS_HIGH_CHAMBER - 0.2, true)) //specimen will hit submersible
                                .waitSeconds(1.7)
                                .afterTime(1.4, new armToReadyHangAct())//get arm in position ready for hanging specimen
                                .strafeTo(new Vector2d(hangSpecimenPos.x, hangSpecimenPos.y - 0.8 * (j - 1)))
                                .turnTo(newStartPose.heading)
                                .build()
                );

                //sleep(3000);

                Logging.log("drive heading after strafe to specimen # %s : %2f", j, drive.pose.heading.log());

                //adjust to sensor dist
                sensorDist = distSensor.getDistance(DistanceUnit.INCH);
                shiftDelta = sensorDist - Params.HIGH_CHAMBER_DIST;
                shiftDelta = (shiftDelta > 12)? 12 : ((shiftDelta < -12)? -12 : shiftDelta);
                Logging.log("drive pose before distance");
                Logging.log(" X position = %2f, Y position = %2f ", drive.pose.position.x, drive.pose.position.y);
                Logging.log("specimen # %s sensor dist = %2f, shift delta = %2f", j, sensorDist, shiftDelta);

                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .strafeToConstantHeading(new Vector2d(drive.pose.position.x + shiftDelta , drive.pose.position.y))
                                .build()
                );
                Logging.log("after adjust X position = %2f, Y position = %2f ", drive.pose.position.x, drive.pose.position.y);

                //put specimen on high chamber
                intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER);
                sleep(200);

                //release specimen and lower arm to clear high chamber
                intake.setFingerPosition(intake.FINGER_OPEN);
                intake.setArmPosition(intake.ARM_POS_GRAB_SAMPLE_BACK + 400);
                intake.setKnucklePosition(intake.KNUCKLE_POS_HIGH_CHAMBER - 0.2);
                sleep(100);
            }

            Logging.log("hanging specimen complete");

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .afterTime(0.3, new intakeAct(intake.ARM_POS_DROP_SAMPLE, intake.WRIST_POS_NEUTRAL, intake.KNUCKLE_POS_DROP_SAMPLE, true))
                            .strafeToLinearHeading(obsZone, Math.toRadians(-135))
                            .build()
            );


            //sleep(18000);
            /* start for second specimen
            //back to observation zone for next specimen
            intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER - 700);
            intake.setKnucklePosition(intake.KNUCKLE_POS_HIGH_CHAMBER + 0.15);
            sleep(300);
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .setReversed(true)
                            .afterTime(1.0, new armToPickupSpecimen()) // get arm ready for second specimen pick up
                            .strafeToLinearHeading(specimenLineUpPos.position, specimenLineUpPos.heading) // spline to side wall for second specimen
                            //.strafeTo(new Vector2d(specimenLineUpPos.position.x, specimenLineUpPos.position.y))
                            //.turnTo(pickUpSpecimenPos.heading) // fine turn heading before pickup first specimen
                            .strafeToLinearHeading(pickUpSpecimenPos.position, specimenLineUpPos.heading)
                            .build()
            );

            Logging.log("after 2nd specimen pick up heading: %2f", Math.toDegrees(drive.pose.heading.log()));
            //pickup second specimen
            intake.setKnucklePosition(intake.KNUCKLE_POS_HIGH_CHAMBER - 0.032);
            sleep(100); //200
            intake.setFingerPosition(intake.FINGER_CLOSE);
            sleep(100); // 200
            intake.setArmPosition(intake.ARM_POS_BACK);
            sleep(200);

            //strafe to hanging second specimen
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            // forward arm during strafe
                            //.afterTime(1.9, new intakeAct(intake.ARM_POS_HIGH_CHAMBER, intake.WRIST_POS_HIGH_CHAMBER, 0)) // forward arm to hang first specimen
                            //.strafeToLinearHeading(hangSpecimenPos, 0)
                            //.strafeToLinearHeading(new Vector2d(hangSpecimenPos.x - 0.1 * Params.HALF_MAT, hangSpecimenPos.y + Params.CHASSIS_HALF_WIDTH), 0)
                            .strafeToLinearHeading(new Vector2d(hangSpecimenPos.x - 0.05 * Params.HALF_MAT, hangSpecimenPos.y - Params.CHASSIS_HALF_WIDTH + 8.0), 0)
                            .build()
            );

            // back arm after hanging the second specimen\=
            // -            sensorDist = distSensor.getDistance(DistanceUnit.INCH);
            shiftDelta = sensorDist - Params.HIGH_CHAMBER_DIST;
            shiftDelta = (shiftDelta > 2)? 2 : ((shiftDelta < -2)? -2 : shiftDelta);
            Logging.log("drive pose before distance");
            Logging.log(" X position = %2f, Y position = %2f ", drive.pose.position.x, drive.pose.position.y);
            Logging.log("third specimen sensor dist = %2f, shift delta = %2f", sensorDist, shiftDelta);

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToConstantHeading(new Vector2d(drive.pose.position.x + shiftDelta , drive.pose.position.y))
                            .build()
            );
            Logging.log("after adjust X position = %2f, Y position = %2f ", drive.pose.position.x, drive.pose.position.y);

            sleep(800);
            intake.setFingerPosition(intake.FINGER_OPEN);
            intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER - 700);
            intake.setKnucklePosition(intake.KNUCKLE_POS_HIGH_CHAMBER + 0.15);

            // back to obs zone for parking
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .setReversed(true) // for parking
                            .strafeTo(parkObz)
                            .build()
            );
            intake.setArmPosition(intake.ARM_POS_GRAB_SAMPLE);
            intake.setFingerPosition(intake.FINGER_SPECIMEN_CLOSE);
            intake.setKnucklePosition(intake.KNUCKLE_POS_HANGING);
            */
        }
        Logging.log("X position = %2f, Y position = %2f", drive.pose.position.x, drive.pose.position.y);
    }

    //action for arm flip to hang
    private class armToReadyHangAct implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER_READY);
            intake.setWristPosition(intake.WRIST_POS_HIGH_CHAMBER);
            intake.setKnucklePosition(intake.KNUCKLE_POS_HIGH_CHAMBER);
            return false;
        }
    }

    //action for arm to pick up neutral sample
    private class armToPickUpPos implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setKnucklePosition(intake.KNUCKLE_POS_PICKUP_SAMPLE_BACK);
            sleep(100);
            intake.setArmPosition(intake.ARM_POS_GRAB_SAMPLE_BACK);
            intake.setWristPosition(intake.WRIST_POS_GRAB_SAMPLE_BACK);
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
        public intakeAct(double armPos, double wristPos, double knucklePos, boolean fingerPos) {
            arm = (int)armPos;
            wrist = (int)wristPos;
            knuckle = knucklePos;
            finger = fingerPos;
        }

        private final int arm;
        private final int wrist;
        private final double knuckle;
        private final boolean finger;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if (arm < 0) {
                intake.setArmPosition(arm);
            }

            if (wrist >= 0) {
                intake.setWristPosition(wrist);
            }

            if (knuckle > 0) {
                intake.setKnucklePosition(knuckle);
            }

            if (finger) {
                intake.fingerServoClose();
            } else {
                intake.fingerServoOpen();
            }
            return false;
        }
    }

    private void updateProfileAccel(boolean slowMode) {
        if (slowMode) {
            MecanumDrive.PARAMS.minProfileAccel = -25;
            MecanumDrive.PARAMS.maxProfileAccel = 40;
        } else {
            MecanumDrive.PARAMS.minProfileAccel = -35;
            MecanumDrive.PARAMS.maxProfileAccel = 60;
        }
    }
}
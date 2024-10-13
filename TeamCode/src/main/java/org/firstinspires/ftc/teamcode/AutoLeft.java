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
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.easyopencv.OpenCvCamera;

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

@Autonomous(name="Left side auto", group="Concept")
//@Disabled
public class AutoLeft extends LinearOpMode {

    /** 1 for Red Front, 2 for Red back, 3 for Blue Front, and 4 for Blue back
     */
    public int startLoc = 1;
    /** blue: 1; red: -1
     */
    private int blueOrRed = 1; // blue: 1; red: -1
    /** front: 1; back -1
     */
    private int frontOrBack = 1; // front: 1; back -1
    /**
     * Parking location: "1" - right of backdrop; "-1" - left of backdrop.
     */
    public int leftOrRight = -1; // -1) means te robot parks near the blue side no matter the autonomous, right(1) is the opposite
    /**
     * blue: 1,2,3; red: 4,5,6
     */
    private int desiredTagNum = 0; // blue: 1,2,3; red: 4,5,6
    private int checkStatus = 1;
    final private double BUCKET_SHIFT = 2.0; // yellow pixel is in the right bucket.
    // USE LATER: boolean debug_flag = true;

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private intakeUnit intake;

    //private SlidersWith2Motors slider;
    private MecanumDrive drive;

    private Servo DroneServo;

    // camera and sleeve color
    ObjectDetection.PropSide propLocation = ObjectDetection.PropSide.UNKNOWN;
    ObjectDetection propDetect;
    OpenCvCamera camera;
    String webcamName = "Webcam 1";
    boolean isCameraInstalled = true;
    // sensing april tag tools
    private AprilTagTest tag = null;

    // road runner variables
    Pose2d startPose;
    Pose2d newStartPose;

    final int WAIT_ALLIANCE_SECONDS = 6;

    /**
     * Set robot starting location on the field:
     * 1 for Red Front, 2 for Red back, 3 for Blue Front, and 4 for Blue back
     */
    public void setRobotLocation() {
        startLoc = 1;
        //leftOrRight = -1;
    }

    /**
     * Set robot starting position, and blueOrRed, frontOrBack variables:
     * @param startLocation : the value of robot location in the field.
     *                      1 for Red Front, 2 for Red back,
     *                      3 for Blue Front, and 4 for Blue back
     */
    private void setStartPoses(int startLocation) {
        // road runner variables
        switch(startLocation) {
            case 1: // left
                leftOrRight = 1;
                break;

            case 2: // right
                leftOrRight = -1;
                break;


        }
        startPose = new Pose2d((6 * Params.HALF_MAT - Params.CHASSIS_HALF_LENGTH - Params.CHASSIS_START_EXTRA) * blueOrRed,
                //Params.HALF_MAT + (3 * Params.HALF_MAT - Params.CHASSIS_HALF_WIDTH) * frontOrBack,
                Params.HALF_MAT + 2 * Params.HALF_MAT * frontOrBack,
                Math.toRadians(90.0 + 90.0 * blueOrRed));
        newStartPose = new Pose2d((-6 * Params.HALF_MAT + Params.CHASSIS_LENGTH / 2),(leftOrRight * Params.CHASSIS_HALF_WIDTH),0);

    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        Logging.log("Status - Initialized");

        //DroneServo = hardwareMap.get(Servo.class, "Drone");
        //DroneServo.setPosition(Params.DRONE_START);

        setRobotLocation();

        setStartPoses(startLoc);

        // use slow mode if starting from front
        updateProfileAccel(frontOrBack > 0);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        propDetect = new ObjectDetection();

        if (startLoc <= 2) {
            propDetect.setColorFlag(ObjectDetection.ColorS.RED);
        } else {
            propDetect.setColorFlag(ObjectDetection.ColorS.BLUE);
        }

        if (isCameraInstalled) {
            //camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(
                    //WebcamName.class, webcamName), cameraMonitorViewId);

            //camera.setPipeline(propDetect);

            /*camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                    Logging.log("Start stream to detect sleeve color.");
                }

                @Override
                public void onError(int errorCode) {
                    Logging.log("Start stream error.");
                    telemetry.addData("Start stream to detect sleeve color.", "error");
                    telemetry.update();
                }
            });

             */
        }

        // init drive with road runner
        drive = new MecanumDrive(hardwareMap, newStartPose);
        Params.startPose = newStartPose; // init storage pose.
        Params.blueOrRed = blueOrRed;

        intake = new intakeUnit(hardwareMap, "Arm", "Wrist", "Finger");
        //intake.setArmModeRunToPosition(0);

        //slider = new SlidersWith2Motors();

        //slider.init(hardwareMap, "sliderRight", "sliderLeft");

        //slider.resetEncoders();

        runtime.reset();
        while ((ObjectDetection.PropSide.UNKNOWN == propLocation) &&
                ((runtime.seconds()) < 3.0)) {
            propLocation = propDetect.getPropPos();
        }

        int spikeMarkLoc = 1; // 1 for left, 2 for center, and 3 for right

        while (!isStarted()) {
            propLocation = propDetect.getPropPos();
            sleep(10);
            telemetry.addData( ((blueOrRed >0)? "Blue - " : "Red - "),((frontOrBack >0)? "front" : "back"));

            //telemetry.addData("Detected Prop location: ", propLocation);
            //telemetry.addData("Arm", "position = %d", intake.getArmPosition());

            telemetry.update();
        }

        switch (propLocation) {
            case LEFT:
                spikeMarkLoc = 1;
                break;
            case CENTER:
            case UNKNOWN:
                spikeMarkLoc = 2;
                break;
            case RIGHT:
                spikeMarkLoc = 3;
                break;
        }
        desiredTagNum = spikeMarkLoc + ((blueOrRed > 0)? 0 : 3); // blue: 1,2,3; red: 4,5,6
        checkStatus = desiredTagNum * frontOrBack;

        tag = new AprilTagTest(drive, hardwareMap, desiredTagNum, webcamName);

        // bulk reading setting - auto refresh mode
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        intake.setFingerPosition(0.0);

        intake.setWristPosition(0.95);

        waitForStart();

        intake.setWristPosition(0.95);

        intake.setFingerPosition(intake.FINGER_CLOSE);

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
        Logging.log("Auto start wrist pos: %2f", intake.getWristPosition());
        Logging.log("Status - Start auto core");
        Vector2d startArmFlip = new Vector2d(startPose.position.x - blueOrRed * 6, startPose.position.y);

        double pausePoseY = -2 * Params.HALF_MAT - 6;
        Vector2d vMatCenter = new Vector2d(blueOrRed * (3 * Params.HALF_MAT), startPose.position.y);
        Vector2d vParkPos = new Vector2d(blueOrRed * 3 * Params.HALF_MAT - 2 * leftOrRight * Params.HALF_MAT, -3.2 * Params.HALF_MAT);
        Vector2d vBackdrop = new Vector2d(blueOrRed * 3 * Params.HALF_MAT, -4 * Params.HALF_MAT);

        Vector2d vAprilTag = null;

        //new stuff

        //hang specimen
        //Vector2d hangSpecimen = new Vector2d(- 3.5 * Params.HALF_MAT, 0);
        Vector2d armFlip = new Vector2d(-4.4 * Params.HALF_MAT, newStartPose.position.y);
        //Vector2d retractArm = new Vector2d(armFlip.x - Params.HALF_MAT, armFlip.y);

        //grab
        Vector2d changeHeadingForPickup = new Vector2d(- 3.5 * Params.HALF_MAT, 1.85 * Params.HALF_MAT);
        Vector2d driveForwardToPickup = new Vector2d(- 3.5 * Params.HALF_MAT, 2.65 * Params.HALF_MAT);
        Vector2d placeSample = new Vector2d(- 4.6 * Params.HALF_MAT, 4.6 * Params.HALF_MAT);

        Vector2d splineThirdSample = new Vector2d(- 2.2 * Params.HALF_MAT, 3 * Params.HALF_MAT);

        //ascent level 1
        Vector2d parkStepOne = new Vector2d(- 4 * Params.HALF_MAT,  4 * Params.HALF_MAT);
        Vector2d parkStepTwo = new Vector2d(parkStepOne.x + 3 * Params.HALF_MAT, parkStepOne.y);
        Vector2d parkStepThree = new Vector2d(parkStepTwo.x, 1.8 * Params.HALF_MAT);

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
        sleep(1700);

        //release specimen and raise arm to clear high chamber
        intake.setFingerPosition(intake.FINGER_OPEN);
        sleep(100);
        intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER - 600);

        //Go to pick up neutral sample
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .afterTime(0.4, new armToPickUpPos())
                        .splineTo(changeHeadingForPickup, Math.toRadians(58))
                        .strafeTo(driveForwardToPickup)
                        .build()
        );
        sleep(100);
        intake.setFingerPosition(intake.FINGER_CLOSE);
        sleep(200);
        intake.setArmPosition(intake.ARM_POS_LOW_BUCKET);
        sleep(200);

        //place sample in bucket
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .afterTime(0.2, new armToDropSampleAct())
                        .splineToLinearHeading(new Pose2d(placeSample, Math.toRadians(135)), Math.toRadians(135))
                        .build()
        );
        sleep(200);
        intake.setFingerPosition(intake.FINGER_OPEN);
        sleep(200);

        //pick up second sample
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .afterTime(0.4, new armToPickUpPos())
                        .splineToLinearHeading(new Pose2d(changeHeadingForPickup.x + 0.3 * Params.HALF_MAT, changeHeadingForPickup.y + 0.85 * Params.HALF_MAT, Math.toRadians(65)), Math.toRadians(90))
                        .strafeTo(new Vector2d(driveForwardToPickup.x + 0.2 * Params.HALF_MAT, driveForwardToPickup.y + 0.85 * Params.HALF_MAT))
                        .build()
        );
        sleep(100);
        intake.setFingerPosition(intake.FINGER_CLOSE);
        sleep(200);
        intake.setArmPosition(intake.ARM_POS_LOW_BUCKET);
        sleep(200);

        //place second sample in bucket
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .afterTime(0.2, new armToDropSampleAct())
                        .splineToLinearHeading(new Pose2d(placeSample, Math.toRadians(135)), Math.toRadians(135))
                        .build()
        );
        sleep(200);
        intake.setFingerPosition(intake.FINGER_OPEN);
        sleep(200);

        //pick up third sample
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .afterTime(0.3, new armToPickUpPos())
                        .splineToLinearHeading(new Pose2d(splineThirdSample, Math.toRadians(90)), Math.toRadians(90))
                        .build()
        );

        //place third sample in bucket
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .afterTime(0.2, new armToDropSampleAct())
                        .splineToLinearHeading(new Pose2d(placeSample, Math.toRadians(135)), Math.toRadians(135))
                        .build()
        );
        sleep(200);
        intake.setFingerPosition(intake.FINGER_OPEN);
        sleep(200);

        //Go to ascent level 1
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .afterTime(0.2, new armToParkingAct())
                        .splineToLinearHeading(new Pose2d(parkStepThree, Math.toRadians(-90)), Math.toRadians(-90))
                        .build()
        );
        sleep(100);


        /*
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeTo(parkStepTwo)
                        .strafeTo(parkStepThree)
                        .build()
        );
         */

        Logging.log("X position = %2f, Y position = %2f", drive.pose.position.x, drive.pose.position.y);

        /*
        if (blueOrRed > 0) {
            vAprilTag = new Vector2d(vBackdrop.x + (2 - desiredTagNum) * Params.BACKDROP_SIDEWAYS, vBackdrop.y);
        } else {
            vAprilTag = new Vector2d(vBackdrop.x + (5 - desiredTagNum) * Params.BACKDROP_SIDEWAYS, vBackdrop.y);
        }
        Vector2d vCheckingAprilTagPose = new Vector2d(vAprilTag.x, vAprilTag.y + Params.HALF_MAT);
        Vector2d vDropYellow = new Vector2d(vAprilTag.x + BUCKET_SHIFT, vAprilTag.y + 2.5); // 3 seem good

        Vector2d vDropPurple = null;
        double xDelta = -7.0;
        double yDelta = 10.0;

        switch (checkStatus) {
            case 5:
            case -5:
            case 2:
            case -2:
                // pass the test
                xDelta = 8;
                yDelta = 0;
                break;
            case -1:
            case 4:
                // pass the test
                xDelta = 12.0;
                yDelta = blueOrRed * -9.0;
                break;
            case -3:
            case -4:
                xDelta = -2; // 0;
                yDelta = -7;
                startArmFlip = new Vector2d(blueOrRed * (3 * Params.HALF_MAT + xDelta), startPose.position.y - 15);
                break;
            case 1:
            case 6:
                // near gate cases
                xDelta = -5; // 0;
                yDelta = 9;
                startArmFlip = new Vector2d(blueOrRed * (3 * Params.HALF_MAT + xDelta), startPose.position.y + 15);
                break;
            case 3:
            case -6:
                // pass the test
                xDelta = 12.0;
                yDelta = blueOrRed * 11.5;
                break;
        }
        vDropPurple = new Vector2d(blueOrRed * (3 * Params.HALF_MAT + xDelta), startPose.position.y + yDelta);

        Logging.log("check status = %d, xDelta = %.2f, yDelta = %.2f ", checkStatus, xDelta, yDelta);
        logVector("Back drop pose", vBackdrop);
        logVector("April tag", vAprilTag);

        logRobotHeading("robot drive: before strafe");
        logVector("robot drive: start position", startPose.position);

        intake.setArmCountPosition(intake.ARM_POS_AUTO);
        // move forward
        Actions.runBlocking(
                drive.actionBuilder(startPose)
                        .strafeTo(startArmFlip) // drive forward several inch to leave wall
                        .build()
        );

        logVector("robot drive: arm to push pose", drive.pose.position);
        logVector("robot drive: start Arm Flip pose required", startArmFlip);

        double tAngle = Math.PI / 2.0 * frontOrBack * blueOrRed + 0.0001; // add 0.0001 to avoid 0 degree turning.
        Logging.log("frontOrBack = %d, blueOrRed = %d, turn angle = %f", frontOrBack, blueOrRed, tAngle);

        // Near gate cases
        if (((6 == checkStatus) || (-3 == checkStatus) || (1 == checkStatus) || (-4 == checkStatus)) &&
                (tAngle != 0.0)) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .turn(tAngle)
                            .build()
            );
            logRobotHeading("robot drive: after turn before arm flip");
            logVector("robot drive: after turn pose starting Arm Flip required", drive.pose.position);
            logVector("robot drive: after turn pose starting Arm Flip required", startArmFlip);
        }


        intake.pushPropPose();
        sleep(2000);

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeTo(vDropPurple)
                        .build()
        );

        logVector("robot drive: drop purple pose", drive.pose.position);
        logVector("robot drive: drop purple pose required", vDropPurple);

        // drop off the purple pixel by arm and wrist actions
        dropPurpleAction();

        double armPower = intake.armMotor.getPower();
        intake.armMotor.setPower(0.1); // use slow speed

        intake.underTheBeam();

        sleep(300);
        intake.armMotor.setPower(armPower);
        intake.switchServoClose();

        if(checkStatus == 2 || checkStatus == 5 ||
                4 == checkStatus || 3 == checkStatus) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            // move away from gate, and avoid stamp on purple pixel
                            .strafeTo(new Vector2d(vMatCenter.x + blueOrRed * 4, vMatCenter.y + 1.5 * Params.HALF_MAT))
                            .build()
            );//strafe several inches left to avoid hitting the beam
        }

        // there is a bug somewhere in turn() function when using PI/2, it actually turn PI
        double turnAngleToDrop = 0;
        if ((-4 == checkStatus) || (-3 == checkStatus)) {
            turnAngleToDrop = -blueOrRed * (Math.PI + 0.0001);

            // move back a little bit before turn to avoid hitting gate
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .lineToYConstantHeading(drive.pose.position.y - 5)
                            .build());
            logVector("robot drive: back after drop purple", drive.pose.position);
            logVector("robot drive: back after drop purple required", vDropPurple);
        } else {
            turnAngleToDrop = (Math.PI / 2) * blueOrRed + 0.0001;
        }

        Logging.log("turn angle = %f", turnAngleToDrop);


        if ((6 != checkStatus) && (1 != checkStatus) && (turnAngleToDrop != 0.0)) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .turn(turnAngleToDrop)
                            .build());
            logRobotHeading("robot drive: turn after drop purple");
            logVector("robot drive: turn after drop purple", drive.pose.position);
            logVector("robot drive: turn after drop purple required", vDropPurple);
        }

        // move to the center of second mat to go through gate.
        if (frontOrBack > 0) {
            // add 2 inch to avoid drive on purple pixel, always 2 inch more to right
            Vector2d driveThroughGate = null;
            if (1 == checkStatus || 6 == checkStatus) {
                driveThroughGate = new Vector2d(vMatCenter.x - 2, vMatCenter.y);
            }
            else {
                driveThroughGate = new Vector2d(vMatCenter.x, vMatCenter.y);
            }
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeTo(driveThroughGate)
                            .build()
            );

            logVector("robot drive: drive.pose move to 2nd mat center", drive.pose.position);
            logVector("robot drive: move to 2nd mat center required", driveThroughGate);
        }

        if (frontOrBack < 0) {
            intake.setArmCountPosition(intake.ARM_POS_CAMERA_READ  + 20); // lift arm to avoid blocking camera
        }
        // fine tune heading angle

        double turnAngle = -drive.pose.heading.toDouble() - Math.PI / 2 + 0.0001;
        turnAngle = (turnAngle != 0.0)? turnAngle : 0.0001;
        Logging.log("turn angle = %f", turnAngle);
        Actions.runBlocking(
                new ParallelAction(
                        // Paral 1. turn on camera for april tag detect
                        new TurnOnCamera(),

                        // Paral 2.
                        (frontOrBack > 0) ?
                                (new SequentialAction(
                                        // Seq a. fine tune heading angle before long travel
                                        drive.actionBuilder(drive.pose)
                                                .turn(turnAngle)
                                                .build(),

                                        // Seq b. waiting alliance move out the way if at front side
                                        new SleepAction((frontOrBack > 0) ? WAIT_ALLIANCE_SECONDS : 0),

                                        // Seq c. strafe to april tag pose to check april tag
                                        drive.actionBuilder(drive.pose)
                                                .lineToYConstantHeading(pausePoseY)
                                                .strafeTo(vCheckingAprilTagPose)
                                                .build())
                                ) : (
                                drive.actionBuilder(drive.pose)
                                        .strafeTo(vCheckingAprilTagPose)
                                        .build()
                        )
                )
        );
        logRobotHeading("robot drive: fine turn for heading correction");
        logVector("robot drive: drive.pose check april tag", drive.pose.position);
        logVector("robot drive: check april tag required", vCheckingAprilTagPose);
        logRobotHeading("robot drive: check april tag");

        if (intake.getArmPosition() > intake.ARM_POS_CAMERA_READ) {
            intake.setArmCountPosition(intake.ARM_POS_CAMERA_READ); // lift arm to avoid blocking camera
            sleep(500);
        }
        Logging.log("Autonomous - Start April tag detect");
        Pose2d aprilTagPose = tag.updatePoseAprilTag(desiredTagNum);
        logVector("robot drive: april tag location from camera", aprilTagPose.position);
        logVector("robot drive: drop yellow pose required before adjust", vDropYellow);

        // if can not move based on April tag, move by road runner encode.
        if (tag.targetFound) {
            // adjust yellow drop-off position according to april tag location info from camera
            vDropYellow = new Vector2d(drive.pose.position.x - aprilTagPose.position.x + BUCKET_SHIFT,
                    drive.pose.position.y - aprilTagPose.position.y + Params.AUTO_DISTANCE_TO_TAG);
            logVector("robot drive: drop yellow pose required after april tag adjust", vDropYellow);
        }
        else {
            if(-3 == checkStatus || -4 == checkStatus) {
                // adjust yellow drop-off position according to testing results
                vDropYellow = new Vector2d(vDropYellow.x, vDropYellow.y - 1.0); // advance 1 inch more for -3/-4 cases
                logVector("robot drive: drop yellow pose required after adjust for -3/-4", vDropYellow);
            }
            Logging.log("Can not found required AprilTag to drop yellow pixel");
        }

        // lower arm for back station
        intake.readyToDropYellow((frontOrBack > 0)? intake.ARM_POS_DROP_YELLOW : intake.ARM_POS_DROP_YELLOW_BACK);

        // shift to AprilTag
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeTo(vDropYellow)
                        .build()
        );
        logVector("robot drive: drive.pose drop yellow", drive.pose.position);
        logVector("robot drive: check drop yellow required", vDropYellow);

        logVector("robot drive: april tag required", vAprilTag);

        // drop pixel
        dropYellowAction();

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .lineToYConstantHeading(vParkPos.y) //move back a little bit to left backdrop board.
                        .strafeTo(vParkPos)
                        .build()
        );
        logVector("robot drive: drive.pose parking", drive.pose.position);
        logVector("robot drive: parking required", vParkPos);

         */
    }

    //action for arm flip to hang
    private class armFlipToHangAct implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER);
            intake.setWristPosition(intake.WRIST_POS_HIGH_CHAMBER);
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

    //action for arm to drop sample
    private class armToDropSampleAct implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setArmPosition(intake.ARM_POS_LOW_BUCKET);
            intake.setWristPosition(intake.WRIST_POS_LOW_BUCKET);
            return false;
        }
    }


    /*
    private void dropPurpleAction() {
        // 1. arm and wrist at correct positions
        intake.readyToDropPurple();
        sleep(100);

        // 2. open switch
        intake.setWristServoPosition(intake.SWITCH_LEFT_RELEASE);
        sleep(1000);
    }
    private void dropYellowAction(){
        intake.setArmServoPosition(intake.SWITCH_RIGHT_RELEASE);
        sleep(500);
        intake.setArmCountPosition(intake.getArmPosition() - 500);
        sleep(500);
    }



    private void logVector(String sTag, Vector2d vXY) {
        String vectorName = vXY.toString();
        Logging.log("%s: %s", sTag, vectorName);
    }

    private void logRobotHeading(String sTag) {
        Logging.log("%s drive.pose: %.2f", sTag, Math.toDegrees(drive.pose.heading.log()));
        //Logging.log("%s imu: %.2f", sTag, drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - Math.toDegrees(startPose.heading.log()));
    }

    private class TurnOnCamera implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            tag.initAprilTag();
            return false;
        }
    }

     */

    private void updateProfileAccel(boolean slowMode) {
        if (slowMode) {
            MecanumDrive.PARAMS.minProfileAccel = -20;
            MecanumDrive.PARAMS.maxProfileAccel = 30;
        }
    }
}

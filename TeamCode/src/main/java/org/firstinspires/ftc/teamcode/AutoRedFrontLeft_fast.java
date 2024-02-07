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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Version;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;
import java.util.Vector;

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

@Autonomous(name="Red Front Left + 2 white", group="Concept")
//@Disabled
public class AutoRedFrontLeft_fast extends LinearOpMode {

    private double NO_ACT = 100000;
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
    private int desiredTagWhite = 0; // blue: 1,2,3; red: 4,5,6

    private int checkStatus = 1;
    final private double BUCKET_SHIFT = 2.0; // yellow pixel is in the right bucket.
    // USE LATER: boolean debug_flag = true;

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private intakeUnit intake;
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

    /**
     * Set robot starting location on the field:
     * 1 for Red Front, 2 for Red back, 3 for Blue Front, and 4 for Blue back
     */
    public void setRobotLocation() {
        startLoc = 1;
        leftOrRight = -1;
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
            case 1: // red front
                blueOrRed = -1;
                frontOrBack = 1;
                break;

            case 2: // red back
                blueOrRed = -1;
                frontOrBack = -1;
                break;

            case 3: //  blue front
                blueOrRed = 1;
                frontOrBack = 1;
                break;

            case 4: //  blue back
                blueOrRed = 1;
                frontOrBack = -1;
                break;
        }
        startPose = new Pose2d((6 * Params.HALF_MAT - Params.CHASSIS_HALF_LENGTH - Params.CHASSIS_START_EXTRA) * blueOrRed,
                //Params.HALF_MAT + (3 * Params.HALF_MAT - Params.CHASSIS_HALF_WIDTH) * frontOrBack,
                Params.HALF_MAT + 2 * Params.HALF_MAT * frontOrBack,
                Math.toRadians(90.0 + 90.0 * blueOrRed));
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        Logging.log("Status - Initialized");

        DroneServo = hardwareMap.get(Servo.class, "Drone");
        DroneServo.setPosition(Params.DRONE_START);

        setRobotLocation();

        setStartPoses(startLoc);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        propDetect = new ObjectDetection();

        if (blueOrRed < 0) {
            propDetect.setColorFlag(ObjectDetection.ColorS.RED);
        } else {
            propDetect.setColorFlag(ObjectDetection.ColorS.BLUE);
        }

        if (isCameraInstalled) {
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(
                    WebcamName.class, webcamName), cameraMonitorViewId);

            camera.setPipeline(propDetect);

            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
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
        }

        // init drive with road runner
        Params.startPose = startPose; // init storage pose.
        Params.blueOrRed = blueOrRed;
        Params.deadWheelOn = MecanumDrive.PARAMS.useDeadWheel;

        intake = new intakeUnit(hardwareMap, "Arm", "Wrist",
                "Finger", "SwitchR", "SwitchL");
        intake.setArmModeRunToPosition(intake.getArmPosition());

        intake.switchServoOpen();

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

            telemetry.addData("Detected Prop location: ", propLocation);
            telemetry.addData("Arm", "position = %d", intake.getArmPosition());
            telemetry.addData(" ---- ", " ---  ");

            telemetry.addData("Arm calibration - -    ", Params.armCalibrated? "Pass!" :" Failed !!!!!");


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

        if (2 == desiredTagNum) {
            desiredTagWhite = 3;
        }
        else if(5 ==desiredTagNum) {
            desiredTagWhite = 4;
        }
        else {
            desiredTagWhite = (desiredTagNum < 3.5)? 2 : 5;
        }
        desiredTagWhite = (desiredTagNum < 3.5)? 2 : 5; // temp center for safe

        // bulk reading setting - auto refresh mode
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        if (opModeIsActive()) {
            drive = new MecanumDrive(hardwareMap, startPose);
            tag = new AprilTagTest(drive, hardwareMap, desiredTagNum, webcamName);

            Logging.log("checkStatus = %d, desiredTagNum = %d", checkStatus, desiredTagNum);
            Logging.log("frontOrBack = %d, blueOrRed = %d", frontOrBack, blueOrRed);

            if (Params.armCalibrated) {
                intake.autonomousInit();
                autonomousCore();

                Params.currentPose = drive.pose; // storage end pose of autonomous
                //intake.parkingPositions(); // Motors are at intake positions at the beginning of Tele-op
                intake.fingerStop();
                sleep(1000);

                camera.closeCameraDevice(); // cost too times at the beginning to close camera about 300 ms
                Logging.log("Autonomous time - total Run Time: " + runtime);
            }
            else {
                telemetry.addData("Arm calibration: ---", "need to be done before starting!");
                telemetry.update();
                sleep(4000);
            }
        }
    }

    private void autonomousCore() {
        autoCore();
        intake.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    private void autoCore() {
        double drop_delta_x = (2 == checkStatus || 5 == checkStatus)? 0 : 3.0;
        double pickup_delta_x = (2 == checkStatus || 5 == checkStatus)? 0 : 1.0;
        double pickup_delta_y = Params.deadWheelOn? 1.1 : 1.0; // temp compensation
        int armShiftCnt = 30;

        Vector2d vParkPos = new Vector2d(blueOrRed * 3 * Params.HALF_MAT - 2 * leftOrRight * Params.HALF_MAT, -4 * Params.HALF_MAT + 2);
        Vector2d vBackdrop = new Vector2d(blueOrRed * 3 * Params.HALF_MAT - blueOrRed * drop_delta_x, -4 * Params.HALF_MAT);

        Vector2d vAprilTag = null;

        if (blueOrRed > 0) {
            vAprilTag = new Vector2d(vBackdrop.x + (2 - desiredTagNum) * Params.BACKDROP_SIDEWAYS, vBackdrop.y);
        } else {
            vAprilTag = new Vector2d(vBackdrop.x + (5 - desiredTagNum) * Params.BACKDROP_SIDEWAYS, vBackdrop.y);
        }

        Vector2d vCheckingAprilTagPose = new Vector2d(vAprilTag.x, vAprilTag.y + Params.HALF_MAT);
        Vector2d vCheckingAprilTag2nd = new Vector2d(vAprilTag.x - (desiredTagWhite - desiredTagNum) * 6, vAprilTag.y + Params.HALF_MAT);

        Vector2d vDropYellow = new Vector2d(vAprilTag.x + BUCKET_SHIFT, vAprilTag.y + 2);
        Vector2d vDropWhite = new Vector2d(vCheckingAprilTag2nd.x, vAprilTag.y - 3);

        Vector2d vDropPurple = null;
        double xDelta = -7.0;
        double yDelta = 10.0;
        double splineTangent = startPose.heading.toDouble();

        double pickWhiteReady_x = blueOrRed * 3.0 * Params.HALF_MAT + BUCKET_SHIFT + blueOrRed * pickup_delta_x;
        double pickWhiteReady_y = 3.5 * Params.HALF_MAT;

        double startTurn1stDrop_x = 3 * Params.HALF_MAT * blueOrRed;

        double startTurn2ndDrop_x = pickWhiteReady_x - 2 * Params.HALF_MAT * blueOrRed; // under gate

        Vector2d vStartTurn1stDrop = new Vector2d(startTurn1stDrop_x, vCheckingAprilTagPose.y + Params.HALF_MAT);
        Vector2d vStartTurn2ndDrop = new Vector2d(startTurn2ndDrop_x, vCheckingAprilTagPose.y + 0.5 *Params.HALF_MAT); // far enough to avoid step on purple

        double dropOffAngle = -Math.PI/2 - 0.04;// * blueOrRed; // 0.04 for turn orientation control
        double pickupAngle = Math.PI/2;

        switch (checkStatus) {
            case 5:
            case -5:
            case 2:
            case -2:
                // pass the test
                xDelta = 9.0;
                yDelta = 2.0 * blueOrRed;
                splineTangent = startPose.heading.toDouble() - Math.PI / 15;
                break;
            case -1:
            case 4:
                // pass the test
                xDelta = 16.0;
                yDelta = blueOrRed * -10.0;
                pickWhiteReady_x = pickWhiteReady_x - blueOrRed * Params.HALF_MAT;
                splineTangent = startPose.heading.toDouble();
                break;
            case -3:
            case -4:
                xDelta = -6; // 0;
                yDelta = -9;
                break;
            case 1:
                // near gate cases
                xDelta = 8; // 0;
                yDelta = 3;
                splineTangent = startPose.heading.toDouble() + blueOrRed * Math.PI / 4;
                break;
            case 6:
                // near gate cases
                xDelta = 12; // 0;
                yDelta = 0;
                splineTangent = startPose.heading.toDouble() + blueOrRed * Math.PI / 4;
                break;
            case 3:
            case -6:
                // pass the test
                xDelta = 16.0;
                yDelta = blueOrRed * 14.0;
                pickWhiteReady_x = pickWhiteReady_x - blueOrRed * Params.HALF_MAT;
                splineTangent = startPose.heading.toDouble();
                break;
        }
        vDropPurple = new Vector2d(blueOrRed * (3 * Params.HALF_MAT + xDelta), startPose.position.y + yDelta);

        Logging.log("check status = %d, xDelta = %.2f, yDelta = %.2f ", checkStatus, xDelta, yDelta);
        logVector("Back drop pose", vBackdrop);
        logVector("April tag", vAprilTag);

        logRobotHeading("robot drive: before strafe");
        logVector("robot drive: start position", startPose.position);
        Logging.log("Autonomous time - robot start moving time: " + runtime);
        logVector("robot drive: drop white position", vDropWhite);


        // spline to drop purple
        if ((2 == checkStatus) || (5 == checkStatus)) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .afterTime(0.001, new CloseCamera()) // close camera
                            .afterDisp(1, new intakeUnitActions(intake.ARM_POS_DROP_PURPLE, intake.WRIST_POS_INTAKE, 0))
                            .splineToSplineHeading(new Pose2d(vDropPurple, splineTangent), splineTangent)
                            .waitSeconds(0.5)  // wait for arm to be in place
                            .build()
            );
        }
        else {
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .afterTime(0.001, new CloseCamera()) // close camera
                            .afterDisp(1, new intakeUnitActions(intake.ARM_POS_DROP_PURPLE, intake.WRIST_POS_INTAKE, 0))
                            .strafeTo(new Vector2d(drive.pose.position.x - blueOrRed * 3, drive.pose.position.y)) // move forward 3 inch
                            .waitSeconds(0.6)  // wait for arm to be in place
                            .splineToSplineHeading(new Pose2d(vDropPurple, splineTangent), splineTangent)
                            .build()
            );
        }

        logVector("robot drive: drop purple pose", drive.pose.position);
        logVector("robot drive: drop purple pose required", vDropPurple);
        Logging.log("Autonomous time - ready to drop purple time: " + runtime);

        // drop off the purple pixel by arm and wrist actions
        dropPurpleAction();
        if ((1 == checkStatus) || (6 == checkStatus)) {
            intake.underTheBeam(); // avoid to hitting rigging bin
        }
        else {
            intake.liftArmToDropPurple();
        }
        Logging.log("Autonomous time - after drop purple time: " + runtime);

        // temp code to move away from purple
        Vector2d pickWhite5 = new Vector2d(pickWhiteReady_x, 4 * Params.HALF_MAT);
        Pose2d pPickWhite1Ready = new Pose2d(pickWhiteReady_x, pickWhiteReady_y, pickupAngle);

        // pickup first white pixel
        if ((2 == checkStatus) || (5 == checkStatus)) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .afterDisp(3, new pickupWhiteAct(intake.ARM_POS_INTAKE5))
                            .splineToLinearHeading(pPickWhite1Ready, pickupAngle)
                            .strafeTo(pickWhite5)
                            .waitSeconds(0.7)
                            //.afterTime(0, new intakeUnitActions(intake.ARM_POS_INTAKE5 + armShiftCnt, NO_ACT, 0))
                            //.waitSeconds(0.3)
                            .build()
            );
        }
        else {
            double angle1 = startPose.heading.toDouble() - drive.pose.heading.toDouble() -  blueOrRed * Math.PI / 2.0; // turn to start heading
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            //.turn(angle1)
                            .afterTime(0.5,new pickupWhiteAct(intake.ARM_POS_INTAKE5))
                            .splineToLinearHeading(pPickWhite1Ready, pickupAngle)
                            .strafeTo(pickWhite5)
                            .waitSeconds(0.4)
                            .afterTime(0, new intakeUnitActions(intake.ARM_POS_INTAKE5 + armShiftCnt, NO_ACT, 0))
                            // back to drop off 2nd round
                            .waitSeconds(0.3)
                            .build()
            );
        }
        Logging.log("Autonomous time - after pickup white time: " + runtime);

        // move back to drop yellow and white, lift arm after through gate
        if ((5 == checkStatus) || (2 == checkStatus)) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .afterTime(0.01, new TurnOnCamera()) // turn on camera for April Tag checking
                            .strafeTo(vStartTurn1stDrop)
                            .afterTime(0, new intakeUnitActions(intake.ARM_POS_CAMERA_READ, intake.WRIST_POS_DROP_YELLOW, 0))
                            .splineToLinearHeading(new Pose2d(vCheckingAprilTagPose, dropOffAngle), dropOffAngle)
                            .build()
            );
        } else {
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .afterTime(0.01, new TurnOnCamera()) // turn on camera for April Tag checking
                            .strafeTo(new Vector2d(vStartTurn2ndDrop.x, 3 * Params.HALF_MAT))
                            .strafeTo(vStartTurn2ndDrop)
                            .splineToLinearHeading(new Pose2d(vCheckingAprilTagPose, dropOffAngle), dropOffAngle)
                            .afterTime(0, new intakeUnitActions(intake.ARM_POS_CAMERA_READ, intake.WRIST_POS_DROP_YELLOW, 0))
                            .build()
            );
        }

        logVector("robot drive: drive.pose check april tag", drive.pose.position);
        logVector("robot drive: check april tag required", vCheckingAprilTagPose);
        logRobotHeading("robot drive: check april tag");

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
            Logging.log("Can not found required AprilTag to drop yellow pixel");
        }

        intake.readyToDropYellow(intake.ARM_POS_DROP_YELLOW);
        if ((5 == checkStatus) || (2 == checkStatus)) {
            sleep(1000); // wait arm down to the position
        }

        // shift to AprilTag
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        //.turn(-drive.pose.heading.toDouble() - Math.PI / 2) // fine turning heading
                        //.strafeTo(vDropYellow)
                        .splineToLinearHeading(new Pose2d(vDropYellow, dropOffAngle), dropOffAngle)
                        .build()
        );
        logVector("robot drive: drive.pose drop yellow", drive.pose.position);
        logVector("robot drive: check drop yellow required", vDropYellow);

        // drop pixel
        dropYellowAction(); // arm is at high position

        Logging.log("Autonomous time - after drop off yellow time: " + runtime);

        // move to pickup second white pixel
        double turnAngle = drive.pose.heading.toDouble() + Math.PI - blueOrRed * 0.03; // 0.03 is control orientation to avoid hitting board;
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .afterTime(1, new pickupWhiteAct(intake.ARM_POS_INTAKE5))
                        .splineToLinearHeading(new Pose2d(vStartTurn2ndDrop, turnAngle), turnAngle)
                        // pickup in right side bucket
                        .splineToLinearHeading(new Pose2d(startTurn2ndDrop_x + (blueOrRed - 1) * BUCKET_SHIFT, pickWhite5.y + pickup_delta_y, pickupAngle), pickupAngle)

                        .waitSeconds(0.3) // make sure the arm is lift after robot in place
                        .afterTime(0, new intakeUnitActions(intake.ARM_POS_INTAKE5 + armShiftCnt, NO_ACT, 0))
                        //.strafeTo(new Vector2d(startTurn2ndDrop_x + (blueOrRed - 1) * delta_x - blueOrRed * 2 * BUCKET_SHIFT, pickWhite5.y)) // move bucket a little bit

                        // back to drop off 2nd round
                        .waitSeconds(0.5)
                        .strafeTo(vStartTurn2ndDrop)
                        .splineToLinearHeading(new Pose2d(vDropWhite, dropOffAngle), dropOffAngle)

                        .afterTime(0.3, new intakeUnitActions(intake.ARM_POS_DROP, intake.WRIST_POS_DROP, 0))
                        .build()
        );

        Logging.log("Autonomous time - after second white pickup time: " + runtime);
        /*
        Logging.log("Autonomous - Start April tag detect 2nd time");
        aprilTagPose = tag.updatePoseAprilTag(desiredTagWhite);
        logVector("robot drive: april tag location from camera", aprilTagPose.position);
        logVector("robot drive: drop white pose required before adjust", vDropWhite);

        // if can not move based on April tag, move by road runner encode.
        if (tag.targetFound) {
            // adjust yellow drop-off position according to april tag location info from camera
            vDropWhite = new Vector2d(drive.pose.position.x - aprilTagPose.position.x + BUCKET_SHIFT,
                    drive.pose.position.y - aprilTagPose.position.y + Params.AUTO_DISTANCE_TO_TAG);
            logVector("robot drive: drop white pose required after april tag adjust", vDropYellow);
        }
        else {
            Logging.log("Can not found required AprilTag to drop white pixel");
        }


         */
        //intake.readyToDropYellow(intake.ARM_POS_DROP_YELLOW);

        // shift to AprilTag
        /*
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeTo(vDropWhite)
                        .build()
        );
        logVector("robot drive: drive.pose drop yellow", drive.pose.position);
        logVector("robot drive: check drop white required", vDropWhite);


         */
        // drop pixels
        dropWhiteAction();

        Logging.log("Autonomous time - after second white drop time: " + runtime);


        // parking
        intake.setArmCountPosition(intake.ARM_POS_READY_FOR_HANG + 200);
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeTo(vParkPos)
                        //.splineTo(startPose.position, -Math.PI)
                        //.strafeTo(new Vector2d(-5 * Params.HALF_MAT, 3 * Params.HALF_MAT))
                        //.turn(- drive.pose.heading.toDouble())
                        .build()
        );
        logVector("robot drive: drive.pose parking", drive.pose.position);
        logVector("robot drive: parking required", vParkPos);
    }

    private void dropPurpleAction() {
        // 1. arm and wrist at correct positions
        intake.readyToDropPurple();
        sleep(200);

        // 2. open switch
        intake.setSwitchLeftPosition(intake.SWITCH_LEFT_RELEASE);
        sleep(500);
    }
    private void dropYellowAction(){
        intake.setSwitchRightPosition(intake.SWITCH_RIGHT_RELEASE);
        sleep(500);
        intake.setSwitchLeftPosition(intake.SWITCH_LEFT_RELEASE);
        sleep(300);
        intake.setArmCountPosition(intake.getArmPosition() - 500);
        sleep(200);
    }

    private void dropWhiteAction(){
        // release both
        intake.switchServoOpen();
        sleep(600);
        intake.setArmCountPosition(intake.getArmPosition() - 500);
        sleep(200);
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

    private class CloseCamera implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            camera.closeCameraDevice();
            return false;
        }
    }

    private void updateProfileAccel(boolean slowMode) {
        if (slowMode) {
            MecanumDrive.PARAMS.minProfileAccel = -15;
            MecanumDrive.PARAMS.maxProfileAccel = 20;
        }
        else {
            MecanumDrive.PARAMS.minProfileAccel = -25;
            MecanumDrive.PARAMS.maxProfileAccel = 45;
        }
    }

    private class intakeUnitActions implements Action {
        public intakeUnitActions(int setArmPos, double setWristPos, long mSecBef) {
            armPos = setArmPos;
            wristPos = setWristPos;
            mSecBeforeAct = mSecBef;
        }

        private int armPos;
        private double wristPos;

        private long mSecBeforeAct;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            sleep(mSecBeforeAct);
            Logging.log("Autonomous time - start arm moving time: " + runtime);

            if (armPos < NO_ACT) {
                intake.setArmCountPosition(armPos);
            }

            if (wristPos < NO_ACT) {
                intake.setWristPosition(wristPos);
            }
            return false;
        }
    }

    private class pickupWhiteAct implements Action {
        public pickupWhiteAct(int setArmPos) {
            armPos = setArmPos;
        }

        private int armPos;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.intakePositions(armPos);
            return false;
        }
    }
}

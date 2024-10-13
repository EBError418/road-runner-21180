/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
 /*
  * PID controller and IMU codes are copied from
  * https://stemrobotics.cs.pdx.edu/node/7268%3Froot=4196.html
  */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
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
 *      Tow slider motors:
 *          "RightSlider"
 *          "LeftSlider"
 *
 *      Tow servo motors:
 *          "ArmServo"
 *          "ClawServo"
 */

@TeleOp(name="Teleop RR", group="Concept")
//@Disabled
public class TeleopRR extends LinearOpMode {
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();

    // chassis
    MecanumDrive mecanum;

    //claw and arm unit
    private intakeUnit intake;

    //private SlidersWith2Motors slider;

    private Servo DroneServo;

    // debug flags, turn it off for formal version to save time of logging
    boolean debugFlag = true;

    private AprilTagTest tag = null;

    final double DESIRED_DISTANCE = 3.0;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        GamePadButtons gpButtons = new GamePadButtons();

        mecanum = new MecanumDrive(hardwareMap, Params.currentPose);
        mecanum.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //tag = new AprilTagTest(mecanum, hardwareMap, 0, "Webcam 1");

        //tag.initAprilTag();

        intake = new intakeUnit(hardwareMap, "Arm", "Wrist",
                "Finger");

        //slider = new SlidersWith2Motors();

        //intake.setArmModeRunToPosition(intake.getArmPosition());

        // bulk reading setting - auto refresh mode
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }



        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Mode", "waiting for start: %s", (Params.blueOrRed > 0) ? "Blue" : "Red");

        telemetry.update();
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //gamepad1 buttons
            gpButtons.checkGamepadButtons(gamepad1, gamepad2);

            double maxDrivePower;
            if (gpButtons.speedUp) {
                maxDrivePower = Params.POWER_HIGH;
            } else if (gpButtons.speedDown) {
                maxDrivePower = Params.POWER_LOW;
            } else {
                maxDrivePower = Params.POWER_NORMAL;
            }
            mecanum.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gpButtons.robotDrive * maxDrivePower,
                            -gpButtons.robotStrafe * maxDrivePower
                    ),
                    -gpButtons.robotTurn * maxDrivePower
            ));

            //slider.init(hardwareMap, "sliderRight", "sliderLeft");

            //slider.setCountPosition(slider.getPosition());
            //slider.runToPosition();

            /*
            // Set position only when button is hit.
            if (gpButtons.wristDown) {
                intake.wristDown();
            }

            // Set position only when button is hit.
            if (gpButtons.wristUp) {
                intake.wristUp();
            }

            if (gpButtons.fingerOuttake) {
                intake.fingerOuttake();
            }

            if (gpButtons.fingerStop) {
                intake.fingerStop();
            }

            if (gpButtons.fingerIntake) {
                intake.fingerIntake();
            }

            if (!gpButtons.speedCtrl) {
                if (gpButtons.armUp) {
                    intake.armLiftAcc();
                }

                if (gpButtons.armDown) {
                    intake.armDownAcc();
                }
            } else {
                if (gpButtons.armUp) {
                    intake.armUp();
                }

                if (gpButtons.armDown) {
                    intake.armDown();
                }
            }

            if (gpButtons.armBeamPosition) {
                intake.underTheBeamIntake();
            }

            if (gpButtons.readyToIntake) {
                intake.intakePositions(intake.ARM_POS_INTAKE);
            }
            if (gpButtons.readyToIntake2nd) {
                intake.intakePositions(intake.ARM_POS_INTAKE2);
            }
            if (gpButtons.readyToIntake3rd) {
                intake.intakePositions(intake.ARM_POS_INTAKE3);
            }
            if (gpButtons.readyToIntake4th) {
                intake.intakePositions(intake.ARM_POS_INTAKE4);
            }

            if (gpButtons.readyToIntake5th) {
                intake.intakePositions(intake.ARM_POS_INTAKE5);
            }
            if (gpButtons.switchOpen) {
                intake.fingerServoOpen();
            }

            if (gpButtons.dropAndBack) {
                intake.fingerServoOpen();
                moveBack(Params.HALF_MAT * 1.5);
            }

            if(gpButtons.lowDropPos){
                intake.readyToDropYellow(intake.ARM_POS_DROP_YELLOW);
            }

            if (gpButtons.switchDropRight) {
                intake.switchServoDropRight();
            }

            if(gpButtons.switchDropLeft){
                intake.switchServoDropLeft();
            }

            if (gpButtons.switchClose) {
                intake.switchServoClose();
            }

            if (gpButtons.readyToDrop) {
                intake.dropPositions();
            }

            if (gpButtons.readyToHang) {
                intake.setArmCountPosition(intake.ARM_POS_READY_FOR_HANG);
            }

            if (gpButtons.hangingRobot) {
                intake.hangingRobot();
            }

            if (gpButtons.droneLaunch) {
                DroneServo.setPosition(Params.DRONE_LAUNCH);
            }

            if (gpButtons.moveToLeftTag) {
                moveByAprilTag_new(1 + ((Params.blueOrRed > 0) ? 0 : 3));

            }

            if (gpButtons.moveToCenterTag) {
                moveByAprilTag_new(2 + ((Params.blueOrRed > 0)? 0 : 3));
            }

            if (gpButtons.moveToRightTag) {
                moveByAprilTag_new(3 + ((Params.blueOrRed > 0) ? 0 : 3));
            }

            if (gpButtons.goThroughGate) {
                moveForward(6 * Params.HALF_MAT);
            }

            if (gpButtons.moveToFront) {
                lineWithAprilTag(2 + ((Params.blueOrRed > 0) ? 0 : 3));
            }

            if (gpButtons.armReset) {
                intake.resetArmPositions(intake.getArmPosition());
            }

            if (gpButtons.setPos) {
                intake.movingPixelPosition();
            }

            if (gpButtons.testDropYellow) {
                intake.readyToDropYellow(intake.ARM_POS_DROP_YELLOW);
            }

            if (gpButtons.testDropWhite) {
                intake.dropWhitePositions();
            }

             */
            /*
            if (gpButtons.sliderUp){
                //slider.setInchPosition(slider.getInchPosition() + 0.001);
                slider.manualControlPos((gpButtons.sliderUpDown));
            }

            if (gpButtons.sliderDown){
                //slider.setInchPosition(slider.getInchPosition() - 0.001);
                slider.manualControlPos(gpButtons.sliderUpDown);
            }

             */

            if (gpButtons.armBackwards) {
                intake.setArmPosition(intake.getArmPosition() + 50);

            }

            if (gpButtons.armForwards) {
                intake.setArmPosition(intake.getArmPosition() - 50);

            }

            if (gpButtons.wristUp) {
                intake.setWristPosition(intake.getWristPosition() + 0.002);
            }

            if (gpButtons.wristDown) {
                intake.setWristPosition(intake.getWristPosition() - 0.002);
            }

            if (gpButtons.fingerClose) {
                intake.setFingerPosition(intake.FINGER_CLOSE);
            }

            if (gpButtons.fingerOpen) {
                intake.setFingerPosition(intake.FINGER_OPEN);
            }

            if (gpButtons.SpecimenPosOne) {
                intake.setArmPosition(intake.ARM_POS_BEFORE_HANG);
                intake.setWristPosition(intake.WRIST_POS_HIGH_CHAMBER);
            }

            if (gpButtons.SpecimenPosTwo) {
                intake.setArmPosition(intake.ARM_POS_BACK);
                intake.setWristPosition(intake.WRIST_POS_HIGH_CHAMBER);
            }

            if (gpButtons.SpecimenPosThree) {
                intake.setArmPosition(intake.ARM_POS_HIGH_CHAMBER);
                intake.setWristPosition(intake.WRIST_POS_HIGH_CHAMBER);
            }

            if (gpButtons.PickUpSpecimen) {
                intake.setArmPosition(intake.ARM_POS_GRAB_SPECIMEN);
                intake.setWristPosition(intake.WRIST_POS_GRAB_SPECIMEN);
            }

            mecanum.updatePoseEstimate();
            Params.currentPose = mecanum.pose;
            if (debugFlag) {
                // claw arm servo log
                telemetry.addData("Finger", "position %.3f", intake.getFingerPosition());

                telemetry.addData("Wrist", "position %.3f", intake.getWristPosition());

                telemetry.addData("Arm", "position = %.3f", (double)(intake.getArmPosition()));


                //telemetry.addData("Finger", "position %.3f", intake.getFingerPosition());

                //telemetry.addData("switch Right", "position %.3f", intake.getSwitchRightPosition());

                //telemetry.addData("switch Left", "position %.3f", intake.getSwitchLeftPosition());

                telemetry.addData(" ", " ");



                telemetry.addData("heading", " %.3f", Math.toDegrees(mecanum.pose.heading.log()));

                telemetry.addData("location", " %s", mecanum.pose.position.toString());

                //telemetry.addData("motor velocity = ", " %.3f", mecanum.leftFront.getVelocity());

                telemetry.update(); // update message at the end of while loop

            }

        }

        //intake.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // The motor stop on their own but power is still applied. Turn off motor.
    }

    private void logVector(String sTag, Vector2d vXY) {
        String vectorName = vXY.toString();
        Logging.log("%s: %s", sTag, vectorName);
    }

    /*
    private void moveByAprilTag_new(int tagNum) {
        intake.dropPositions();
        sleep(300); // make sure arm is out of camera sight

        Pose2d aprilTagPose = tag.updatePoseAprilTag_new(tagNum);

        // if can not move based on April tag, moved by road runner.
        if (tag.targetFound) {
            mecanum.updatePoseEstimate();
            mecanum.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            Logging.log("yaw in new = %.2f", Math.toDegrees(aprilTagPose.heading.log()));
            logVector("robot drive: distance from camera to april tag", aprilTagPose.position);

            // adjust yellow drop-off position according to april tag location info from camera
            Vector2d desiredMove = new Vector2d(mecanum.pose.position.x - aprilTagPose.position.x,
                    mecanum.pose.position.y - aprilTagPose.position.y);
            logVector("robot drive: before move to tag pose", mecanum.pose.position);

            logVector("robot drive: move to tag pose required", desiredMove);
            logRobotHeading("before moving to april tag");
            // shift to AprilTag
            Actions.runBlocking(
                    mecanum.actionBuilder(mecanum.pose)
                            .strafeToLinearHeading(desiredMove, mecanum.pose.heading.toDouble() + aprilTagPose.heading.toDouble())
                            .build()
            );
            logRobotHeading("after moving to april tag");


            mecanum.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }


    private void lineWithAprilTag(int tagNum) {
        intake.dropPositions();
        sleep(300); // make sure arm is out of camera sight

        /*  // do back when switch to drop automatically.
        Actions.runBlocking(
                mecanum.actionBuilder(mecanum.pose)
                        .lineToY(mecanum.pose.position.y + 1.5 * Params.HALF_MAT)
                        .build()
        );



        //sleep(850);

        // if can not move based on April tag, moved by road runner.
        if (tag.targetFound) {
            mecanum.updatePoseEstimate();
            mecanum.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            Logging.log("tag target found.");
            logVector("robot drive: distance from camera to april tag", aprilTagPose.position);

            // adjust yellow drop-off position according to april tag location info from camera
            Vector2d desiredMove = new Vector2d(mecanum.pose.position.x - aprilTagPose.position.x,
                    mecanum.pose.position.y + 6.5 * Params.HALF_MAT);
            logVector("robot drive: before move to pose", mecanum.pose.position);
            //intake.armMotor.setPower(0.5);
            intake.underTheBeam();
            Actions.runBlocking(
                    mecanum.actionBuilder(mecanum.pose)
                            .strafeToLinearHeading(new Vector2d(desiredMove.x, mecanum.pose.position.y), -Math.PI / 2)
                            .strafeTo(desiredMove)
                            .turn(-Math.PI / 2 * Params.blueOrRed)
                            .build()
            );
            logVector("robot drive: move to tag pose required", desiredMove);
            logRobotHeading("after moving to april tag");
            //intake.armMotor.setPower(1);
            mecanum.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else {
            Logging.log("new command: april tag not detected");
        }
    }
    */

    private void logRobotHeading(String sTag) {
        Logging.log("%s drive.pose: %.2f", sTag, Math.toDegrees(mecanum.pose.heading.log()));
        //Logging.log("%s imu: %.2f", sTag, mecanum.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - Math.toDegrees(Params.startPose.heading.log()));
    }

    private void moveForward(double moveDistance) {
        intake.underTheBeam();
        sleep(300); // make sure arm is out of camera sight

        mecanum.updatePoseEstimate();
        mecanum.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        logVector("robot drive: current drive pose", mecanum.pose.position);
        logRobotHeading("before moving to back area");

        double turnAngle = -mecanum.pose.heading.log() - Math.PI / 2.0;
        turnAngle = (Math.abs(turnAngle) > Math.PI)? (turnAngle - Math.signum(turnAngle) * 2 * Math.PI) :  turnAngle;
        // shift to AprilTag
        Actions.runBlocking(
                mecanum.actionBuilder(mecanum.pose)
                        .turn(turnAngle)
                        .lineToYLinearHeading(mecanum.pose.position.y - moveDistance, -Math.PI / 2.0)
                        .build()
        );
        logRobotHeading("after moving to back area");
        logVector("robot drive: arrive back area, drive pose", mecanum.pose.position);
        mecanum.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void moveBack(double moveDistance) {
        mecanum.updatePoseEstimate();
        mecanum.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        logVector("robot drive: current drive pose", mecanum.pose.position);
        logRobotHeading("before moving to back area");

        // shift to AprilTag
        Actions.runBlocking(
                mecanum.actionBuilder(mecanum.pose)
                        .lineToYLinearHeading(mecanum.pose.position.y + moveDistance, -Math.PI / 2.0)
                        .build()
        );
        logRobotHeading("after moving to back area");
        logVector("robot drive: arrive back area, drive pose", mecanum.pose.position);
        mecanum.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}

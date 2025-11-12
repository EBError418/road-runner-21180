package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto Near Blue 2026", group = "Concept")
public class AutoNearBlue2026 extends LinearOpMode {
    // get the software-hardware links ready
    private final ElapsedTime runtime = new ElapsedTime();

    private MecanumDrive drive;
    private intakeUnit2026 motors;
    private Colored patternDetector;
    public int leftOrRight; // 1 for blue, -1 for red

    private int[] pickupOrder = {1, 2, 3}; // 1: the artifacts row closest to the shooting target

    public void setSide() {
        leftOrRight = 1;
    }
    private double detectedPattern; // limelight detected pattern

    private Vector2d shootPos; // where the robot should shoot
    private double shootHeading; //the direction the robot shoot in

    @Override
    public void runOpMode() {
        setSide();

        // connect the hardware map to color discrimination system and prepare launcher, intake, and trigger
        patternDetector = new Colored(hardwareMap);
        motors = new intakeUnit2026(hardwareMap, "launcher", "intake", "triggerServo");

        // set the starting position at (6 HALFMATS - HALF OF ROBOT LENGTH, HALF OF ROBOT WIDTH [sign depends on sign]) and with heading in reverse
        // where the robot is placed at the start
        Pose2d startPose = new Pose2d(
                (6 * Params.HALF_MAT - Params.CHASSIS_HALF_LENGTH),
                (leftOrRight * Params.CHASSIS_HALF_WIDTH), Math.toRadians(180.0)
        );

        // define shoot position at (HALF_MAT, HALF_MAT), (HALF_MAT, -HALF_MAT), calculate shoot angle based on location x, y
        double shootPosX = 1 * Params.HALF_MAT;
        double shootPosY = leftOrRight * Params.HALF_MAT;
        // made the following polarity change to shootHeading calculation
        shootHeading = Math.toRadians(180.0) + Math.atan2(leftOrRight * (6 * Params.HALF_MAT - Math.abs(shootPosY)), 6 * Params.HALF_MAT - shootPosX);
        shootPos = new Vector2d(shootPosX, shootPosY);

        // set up driving system
        drive = new MecanumDrive(hardwareMap, startPose);

        // init position of trigger
        motors.triggerClose();

        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            run_auto();
        }
    }

    private void run_auto() {
        motors.triggerClose();
        // Run the first leg of the path: move to shooting position while detecting pattern
        updateProfileAccel(true);
        Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose())
                .afterDisp(3 * Params.HALF_MAT, new limeLightCamera()) // start limelight detection after moving 3 half mats
                .afterDisp(3.1 * Params.HALF_MAT, new startLauncherAction()) // start launcher motor after moving 3.1 half mats
                .strafeToConstantHeading(shootPos) // move to shooting position
                .turnTo(shootHeading) // turn to shooting direction
                .build());
        // shoot preload artifacts
        shootArtifacts();

        // the following is a velocity constraint for moving to pick up artifacts
        VelConstraint pickupSpeed = (robotPose, _path, _disp) -> 8.5;

        setupPickupOrder((int)detectedPattern);

        // Loop to go through all 3 rows to pick up artifacts and shoot them
        for (int pickupIndex = 0; pickupIndex < 3; pickupIndex++) {
            Vector2d pickupPos;
            Vector2d pickupEndPos;
            // 23 is the closest row to start position, then 22, then 21, so new if statement below will optimize pathing
            int rowNum = pickupOrder[pickupIndex];

            pickupPos = rowChoose(rowNum);
            // fixed polarity below (there was a double negative sign before)
            pickupEndPos = new Vector2d(pickupPos.x,pickupPos.y + 1.3 * Params.HALF_MAT * Math.signum(pickupPos.y));

            // action for picking up artifacts
            Action actMoveToPickup = drive.actionBuilder(drive.localizer.getPose())
                    // add 4 degree more for row1 according to test results
                    .strafeToLinearHeading(pickupPos, Math.toRadians(90.0*leftOrRight))
                    .build();
            Actions.runBlocking(actMoveToPickup); // ready for pickup artifacts

            //correct heading for first row picking up
            //if (rowNum == 1)
            /*
            {
                Actions.runBlocking(
                        drive.actionBuilder(drive.localizer.getPose())
                                .turnTo(Math.toRadians(90.0))
                                .build()
                );
            }

             */

            // only shooting first and second pickups, no time for the third.
            //if (pickupIndex < 2) {
            if (runtime.milliseconds() < 28.0 * 1000 ) // 2.0 second left for pickup
            {
                // starting intake motor
                motors.startIntake();
                Action actIntake = drive.actionBuilder(drive.localizer.getPose())
                        .strafeToConstantHeading(pickupEndPos, pickupSpeed) // picking up artifacts
                        .build();
                Actions.runBlocking(actIntake); // complete pickup artifacts

                if (pickupIndex < 2) {
                    // only need to go back a little bit for row 2nd and 3rd
                    if (rowNum > pickupOrder[pickupIndex + 1]) {
                        // after pickup, need to go back a bit to avoid obstacles from other rows
                        Action actMoveBack;
                        actMoveBack = drive.actionBuilder(drive.localizer.getPose())
                                // reverse intake to get rid of last artifacts if it is still not picked up
                                // this is for the case that there is additional artifact has been in the robot
                                // which is failed to shoot out.
                                .afterTime(0.01, new revertIntakeAction())
                                .strafeToConstantHeading(pickupPos)
                                .build();
                        Actions.runBlocking(actMoveBack);
                    }

                    Action actMoveToLaunch = drive.actionBuilder(drive.localizer.getPose())
                            .afterTime(0.01, new revertIntakeAction())
                            .strafeToLinearHeading(shootPos, shootHeading)
                            .build();
                    Actions.runBlocking(actMoveToLaunch);

                    // shoot picked up artifacts
                    shootArtifacts();
                }
            }
        }
        // move out of the Triangle
        /*
        Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose())
                .strafeToLinearHeading(new Vector2d(0, leftOrRight*3.8*Params.HALF_MAT), Math.toRadians(180))
                .build());
         */
    }

    // function to shoot 3 artifacts
    private void shootArtifacts() {
        int waitTimeForTriggerClose = 300;
        int waitTimeForTriggerOpen = 700;
        Logging.log("start shooting.");
        // start launcher motor if it has not been launched
        if (motors.getLauncherPower() < 0.1) {
            Logging.log("start launcher motor since it is stopped.");
            motors.startLauncher(motors.firstArtifactPower);
            sleep(waitTimeForTriggerOpen + 500); // waiting time for launcher motor ramp up
        }

        motors.triggerOpen(); // shoot first
        sleep(waitTimeForTriggerClose);
        motors.triggerClose(); //close trigger to wait launcher motor speed up after first launching

        motors.startLauncher(motors.secondArtifactPower);  // use second artifact power
        motors.startIntake(); // start intake motor to move 3rd artifacts into launcher
        sleep(waitTimeForTriggerOpen); // waiting time for launcher motor ramp up
        motors.triggerOpen(); // shoot second

        motors.revertIntake(); // temp reverse intake
        motors.startLauncher(); // reset to use normal power
        int tmpSleep = 100;
        sleep(tmpSleep);
        motors.stopIntake();
        sleep(waitTimeForTriggerClose - tmpSleep);
        motors.triggerClose();
        motors.startIntake(); // start intake again

        sleep(waitTimeForTriggerOpen); // waiting time for launcher motor ramp up
        motors.triggerOpen(); // shoot third
        sleep(waitTimeForTriggerClose);

        motors.triggerClose();
        sleep(waitTimeForTriggerOpen); // waiting time for launcher motor ramp up
        motors.triggerOpen(); // shoot forth in case one artifact left.
        sleep(waitTimeForTriggerClose + 200);

        motors.triggerClose();
        motors.stopLauncher();
        motors.stopIntake();
    }

    // function that chooses the right row based on detected pattern, returns a Vector2d
    private Vector2d rowChoose(double rownumber) {
        return new Vector2d(
                (-rownumber * 2 + 3) * Params.HALF_MAT,
                leftOrRight * (2.3 * Params.HALF_MAT + Params.CHASSIS_HALF_LENGTH / 2)
        );
    }

    // limelight detection action
    private class limeLightCamera implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            for (int i = 0; i < 30; i++) { // check for 30 cycles (~30 milliseconds) to detect pattern
                detectedPattern = patternDetector.returnId();
                Logging.log("pattern  = %f", detectedPattern);
                telemetry.addData("limelight", "Detected Pattern = %f", detectedPattern);
                telemetry.update();
                if ((detectedPattern > 20) && (detectedPattern < 24)) {
                    return false;
                }
                sleep(1);
            }
            detectedPattern = 23; // default pattern if limelight can't detect
            return false;
        }
    }

    // action to start launcher motor
    private class startLauncherAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            Logging.log("start launcher motor.");
            motors.startLauncher(motors.firstArtifactPower);
            return false;
        }
    }

    // action to revert and stop intake
    private class revertIntakeAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            //motors.startIntake(-0.8 * motors.intakePower); // revert with lower power.
            sleep(100);
            motors.stopIntake();
            return false;
        }
    }

    // action to start intake motor
    private class startIntakeAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            Logging.log("start intake motor.");
            motors.startIntake();
            return false;
        }
    }

    // function to update profile acceleration for maxMode (EXPERIMENTAL)
    private void updateProfileAccel(boolean maxMode) {
        if (maxMode) {
            MecanumDrive.PARAMS.maxWheelVel = 70;
            MecanumDrive.PARAMS.maxProfileAccel = 70;
            MecanumDrive.PARAMS.minProfileAccel = -70;
        } else {
            MecanumDrive.PARAMS.maxWheelVel = 65;
            MecanumDrive.PARAMS.maxProfileAccel = 50;
            MecanumDrive.PARAMS.minProfileAccel = -40;
        }
    }

    private void setupPickupOrder(int pt) {
        switch (pt) {
            case 21:
                pickupOrder = new int[]{3, 1, 2};
                break;
            case 22:
                pickupOrder = new int[]{2, 1, 3};
                break;
            case 23:
            default:
                pickupOrder = new int[]{1, 2, 3};
                //pickupOrder = new int[]{3, 1, 2};
                break;
        }
    }

    // For future use when we have an artifact sorter mechanism
    //    private void sortArtifacts(int pattern) {
    //        switch (pattern) {
    //            case 21:
    //                telemetry.addLine("Sorting: GPP");
    //                break;
    //            case 22:
    //                telemetry.addLine("Sorting: PGP");
    //                break;
    //            case 23:
    //                telemetry.addLine("Sorting: PPG");
    //                break;
    //            default:
    //                telemetry.addLine("Sorting: Unknown");
    //                break;
    //        }
    //        telemetry.update();
    //    }
}
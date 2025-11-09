package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Far Blue shoot pickup 2026", group = "Concept")
@Disabled
public class AutoMoveFarBlue2026 extends AutoFarBlue2026 {
    private MecanumDrive drive;
    private intakeUnit2026 motors;

    private final int leftOrRight = 1; // 1 for blue, -1 for red

    Pose2d startPos;
    Vector2d endPose;
    Vector2d shootPos = new Vector2d(6 * Params.HALF_MAT, leftOrRight * Params.HALF_MAT);

    @Override
    public void runOpMode() {
        // Initialize hardware
        motors = new intakeUnit2026(hardwareMap, "launcher", "intake", "triggerServo");

        // Starting pose
        startPos = new Pose2d(
                (6 * Params.HALF_MAT - Params.CHASSIS_HALF_LENGTH * 1.3),
                (leftOrRight * Params.CHASSIS_HALF_WIDTH * 1.3),
                Math.toRadians(145)
        );

        //drive here after launching
        endPose = new Vector2d(- 5 * Params.HALF_MAT, leftOrRight * 4 * Params.HALF_MAT);

        drive = new MecanumDrive(hardwareMap, startPos);

        waitForStart();
        if (opModeIsActive()) {
            run_auto();
        }
    }

    private void run_auto() {
        motors.triggerClose();

        // shoot preload artifacts
        shootArtifacts(true);

        // lower speed when picking up
        VelConstraint pickupSpeed = (robotPose, _path, _disp) -> 8.0;

        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .strafeToLinearHeading(new Vector2d(3 * Params.HALF_MAT, leftOrRight * 3 * Params.HALF_MAT), Math.toRadians(90))
                        .afterTime(0.0001, packet -> {
                            motors.startIntake();
                            return false;
                        })
                        .strafeToConstantHeading(new Vector2d(3 * Params.HALF_MAT, leftOrRight * 6 * Params.HALF_MAT), pickupSpeed)
                        .strafeToConstantHeading(new Vector2d(3 * Params.HALF_MAT, leftOrRight * 2 * Params.HALF_MAT))
                        .strafeToLinearHeading(shootPos, Math.toRadians(145))
                        .afterTime(0.0001, packet -> {
                            shootArtifacts(true);
                            return false;
                        })
                        .strafeToLinearHeading(new Vector2d(-1 * Params.HALF_MAT, leftOrRight * 3 * Params.HALF_MAT), Math.toRadians(90))
                        .afterTime(0.0001, packet -> {
                            motors.startIntake();
                            return false;
                        })
                        .strafeToConstantHeading(new Vector2d(-1 * Params.HALF_MAT, leftOrRight * -1 * Params.HALF_MAT))
                        .strafeToLinearHeading(shootPos, Math.toRadians(145))
                        .afterTime(0.0001, packet -> {
                            shootArtifacts(true);
                            return false;
                        })
                        .build()
        );
    }

    // Shoots 3 balls with same timing as before
//    private void shootArtifacts() {
//        int waitTimeForTriggerClose = 300;
//        int waitTimeForTriggerOpen = 950;
//
//        Logging.log("start shooting.");
//
//        if (motors.getLauncherPower() < 0.1) {
//            Logging.log("start launcher motor since it is stopped.");
//            motors.startLauncherFar();
//            sleep(waitTimeForTriggerOpen + 500);
//        }
//
//        motors.triggerOpen();
//        sleep(waitTimeForTriggerClose);
//        motors.triggerClose();
//
//        motors.startIntake();
//        sleep(waitTimeForTriggerOpen);
//        motors.triggerOpen();
//        sleep(waitTimeForTriggerClose);
//
//        motors.triggerClose();
//        sleep(waitTimeForTriggerOpen);
//        motors.triggerOpen();
//        sleep(waitTimeForTriggerClose + 150);
//
//        motors.triggerClose();
//        motors.stopLauncher();
//        motors.stopIntake();
//    }
}
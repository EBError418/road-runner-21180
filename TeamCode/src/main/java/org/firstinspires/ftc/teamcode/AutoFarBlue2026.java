package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Far Blue shoot 2026", group = "Concept")
public class AutoFarBlue2026 extends Teleop2026 {
    private MecanumDrive drive;
    private intakeUnit2026 motors;

    public int leftOrRight = 1;

    Vector2d endPose;

    public void setSide() {
        leftOrRight = 1;
    }

    @Override
    public void runOpMode() {
        // Initialize hardware
        motors = new intakeUnit2026(hardwareMap, "launcher", "intake", "triggerServo");

        setSide();

        // Starting pose
        Pose2d startPose = new Pose2d(
                (-6 * Params.HALF_MAT + Params.CHASSIS_HALF_LENGTH),
                (leftOrRight * Params.CHASSIS_HALF_WIDTH),
                Math.toRadians(-155)
        );

        //drive here after launching
        endPose = new Vector2d(- 5 * Params.HALF_MAT, leftOrRight * 3 * Params.HALF_MAT);

        drive = new MecanumDrive(hardwareMap, startPose);
        motors.triggerClose();



        waitForStart();
        if (opModeIsActive()) {
            run_auto();
        }
    }

    private void run_auto() {

        shootArtifacts(true);

        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .strafeToLinearHeading(endPose, Math.toRadians(0))
                        .build()
        );
    }

//    // Shoots 3 balls with same timing as before
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
package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer;

public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            while (opModeIsActive()) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x
                        ),
                        -gamepad1.right_stick_x
                ));

                if (gamepad1.a) {
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .turn(Math.PI / 2)
                                    .build());
                }

                if (gamepad1.b) {
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .turn(Math.PI)
                                    .build());
                }

                if (gamepad1.x) {
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .turn(-Math.PI / 2)
                                    .build());
                }

                if (gamepad1.y) {
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .turn(-Math.PI)
                                    .build());
                }

                /*
                sleep(1000);

                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -0.8,
                                -gamepad1.left_stick_x
                        ),
                        -gamepad1.right_stick_x
                ));
                sleep(2000);

                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x
                        ),
                        -gamepad1.right_stick_x
                ));
                sleep(1000);

                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                0.8,
                                -gamepad1.left_stick_x
                        ),
                        -gamepad1.right_stick_x
                ));
                sleep(2000);
                 */

                drive.updatePoseEstimate();
                telemetry.addData("left front pos", drive.leftFront.getCurrentPosition());
                telemetry.addData("left back pos", drive.leftBack.getCurrentPosition());
                telemetry.addData("right front pos", drive.rightFront.getCurrentPosition());
                telemetry.addData("right back pos", drive.rightBack.getCurrentPosition());

                if (drive.localizer instanceof ThreeDeadWheelLocalizer) {
                    ThreeDeadWheelLocalizer dl = (ThreeDeadWheelLocalizer) drive.localizer;
                    telemetry.addData("par0 pos = ", dl.par0.getPositionAndVelocity().position);
                    telemetry.addData("par1 pos = ", dl.par1.getPositionAndVelocity().position);
                    telemetry.addData("perp pos = ", dl.perp.getPositionAndVelocity().position);
                }
                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading", drive.pose.heading);
                telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));

                telemetry.update();

                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            while (opModeIsActive()) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                0.0
                        ),
                        -gamepad1.right_stick_x
                ));

                drive.updatePoseEstimate();

                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
                telemetry.update();

                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
        } else {
            throw new RuntimeException();
        }
    }
}

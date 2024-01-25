package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Gamepad;


/**
 * Game pad buttons design:
 * Game Pad1:
 *  Left stick:
 *      left-right: strafe robot
 *      up-down:    drive root
 *  Right stick:
 *      left-right: turn robot
 *  dpad:
 *      left:       robot movement speed down
 *      right:      robot movement speed up
 *      up:         normal auto pickup the 3rd cone from cone stack
 *      down:       normal auto pickup the 4th and 5th cone from cone stack
 *  X:              robot movement speed down (same as dpad_left)
 *  B:              robot movement speed down (same as dpad_right)
 *  Left Bumper:    normal auto pickup cone from ground
 *  Right bumper:   normal auto drop off cone
 *  Left trigger:   auto pick up cone from cone base then move to high junction, slider lifted to high junction
 *  Right trigger:  auto drop off cone on high junction then move to cone base
 *  Y:              Teapot function - drop off cone on high junction then pick up cone from base, then move to high junction
 *  Back:           Back to cone base just at the beginning of Teleop
 *  A:              open claw to drop off cone manually.*
 * Game pad2:
 *  dpad:
 *      Up:         close the claw to pick up cone
 *      Down:       open claw to drop off cone manually
 *  Right stick:
 *                  left-right: manually control slider up and down
 *  X:              move slider to wall position
 *  A:              move slider to low junction position
 *  B:              move slider to medium junction position
 *  Y:              move slider to high junction position
 *  Right bumper:   move slider to ground junction position
 *  Right trigger:  move slider to ground position
 *  Left stick:
 *      left-right: manually control arm position
 *      up:         move arm to pick up position
 *      down:       move arm to drop off position
 *
 */
public class GamePadButtons {
    //game pad setting
    public float robotDrive;
    public float robotStrafe;
    public float robotTurn;
    public boolean lowDropPos;
    public boolean speedDown;
    public boolean speedUp;
    public boolean armBeamPosition;
    public boolean armUp;
    public boolean armDown;
    public boolean armReset;
    public boolean wristUp;
    public boolean wristDown;

    public boolean fingerOuttake;
    public boolean fingerIntake;
    public boolean fingerStop;
    public boolean dropAndBack;
    public boolean switchOpen;
    public boolean switchDropRight;
    public boolean switchDropLeft;
    public boolean switchClose;
    public boolean speedCtrl;
    public boolean readyToIntake;
    public boolean readyToIntake2nd;
    public boolean readyToIntake3rd;
    public boolean readyToIntake4th;
    public boolean readyToIntake5th;

    public boolean readyToDrop;
    public boolean droneLaunch;
    public boolean hangingRobot;
    public boolean readyToHang;
    public boolean moveToLeftTag;
    public boolean moveToCenterTag;
    public boolean moveToRightTag;
    public boolean moveToFront;

    public boolean goThroughGate;

    public void checkGamepadButtons(@NonNull Gamepad gamepad1, @NonNull Gamepad gamepad2) {
        //game pad 1 buttons

        //driving
        robotDrive = gamepad1.left_stick_y;
        robotStrafe = gamepad1.left_stick_x;
        robotTurn = gamepad1.right_stick_x;

        //speed controls
        speedCtrl = gamepad1.back || gamepad2.back;
        speedDown = speedCtrl;

        // auto moving during Teleop
        //moveToLeftTag = gamepad1.x;
        //moveToCenterTag = gamepad1.y;
        //moveToRightTag = gamepad1.b;
        //moveToFront = gamepad1.a;


        //preset ARM positions for intake
        readyToIntake = gamepad1.left_bumper;
        readyToIntake2nd = gamepad1.dpad_up;
        readyToIntake3rd = gamepad1.dpad_left;
        readyToIntake4th = gamepad1.dpad_right;
        readyToIntake5th = gamepad1.dpad_down;

        // preset ARM positions for drop
        readyToDrop = gamepad1.right_bumper;
        lowDropPos = (gamepad1.right_trigger > 0);

        // game pad 2

        //special presets
        droneLaunch = (gamepad2.right_trigger > 0) && (gamepad2.dpad_right);// || (gamepad1.left_trigger > 0);

        readyToHang = gamepad2.left_bumper;
        hangingRobot = (gamepad2.left_trigger > 0);

        //back switches
        switchOpen = gamepad2.y  || gamepad1.a;
        switchClose = gamepad2.a;
        switchDropRight = gamepad2.b;
        switchDropLeft = gamepad2.x;
        //dropAndBack = gamepad2.right_bumper;
        
        //arms
        armBeamPosition = gamepad2.right_bumper;
        armUp = gamepad2.right_stick_y < 0;
        armDown = gamepad2.right_stick_y > 0;
        armReset = gamepad2.start;

        //wrists
        wristUp = gamepad2.left_stick_y < 0;
        wristDown = gamepad2.left_stick_y > 0;

        //fingers
        fingerIntake =  gamepad2.dpad_down;
        fingerStop =  gamepad2.dpad_left;
        fingerOuttake =  gamepad2.dpad_up;
    }
}

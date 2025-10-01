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
public class GamePadButtons2026 {
    //game pad setting
    public float robotDrive;
    public float robotStrafe;
    public float robotTurn;
    public boolean speedDown;
    public boolean speedUp;
    public boolean speedCtrl;

    public boolean servoStart;
    public boolean servoStop;

    public void checkGamepadButtons(@NonNull Gamepad gamepad1, @NonNull Gamepad gamepad2) {
        //game pad 1 buttons

        //driving
        /*
        robotDrive = gamepad1.left_stick_y;
        robotStrafe = gamepad1.left_stick_x;
        robotTurn = gamepad1.right_stick_x;
         */

        robotDrive = gamepad1.left_stick_y * gamepad1.left_stick_y * gamepad1.left_stick_y;
        robotStrafe = gamepad1.left_stick_x * gamepad1.left_stick_x * gamepad1.left_stick_x;
        robotTurn = gamepad1.right_stick_x * gamepad1.right_stick_x * gamepad1.right_stick_x;

        //speed controls
        speedCtrl = gamepad1.right_bumper || gamepad2.back;
        speedDown = speedCtrl;

        servoStart = gamepad1.dpad_up;
        servoStop = gamepad1.dpad_down;
    }
}

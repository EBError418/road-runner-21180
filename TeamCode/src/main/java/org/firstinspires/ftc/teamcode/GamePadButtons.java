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
    public boolean speedDown;
    public boolean speedUp;
    public boolean wristUp;
    public boolean wristDown;
    public boolean speedCtrl;
    public boolean sliderUp;

    public float sliderUpDown;
    public boolean sliderDown;
    public boolean armForwards;
    public boolean armBackwards;
    public boolean fingerOpen;
    public boolean fingerClose;
    public boolean SpecimenPosOne;
    public boolean SpecimenPosTwo;
    public boolean SpecimenPosThree;

    public boolean PickUpSpecimen;

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
        speedCtrl = gamepad1.back || gamepad2.back;
        speedDown = speedCtrl;

        // auto moving during Teleop
        //moveToLeftTag = gamepad1.x;
        //moveToCenterTag = gamepad1.y;
        //moveToRightTag = gamepad1.b;
        //moveToFront = gamepad1.a;

        //finger
        fingerClose = gamepad2.dpad_up;
        fingerOpen = gamepad2.dpad_down;

        //wrists
        wristUp = gamepad2.right_stick_y < 0;
        wristDown = gamepad2.right_stick_y > 0;

        //arm
        armForwards = gamepad2.left_stick_y > 0;
        armBackwards = gamepad2.left_stick_y < 0;

        //slider
        sliderUp = gamepad2.right_stick_y > 0;
        sliderDown = gamepad2.right_stick_y < 0;
        sliderUpDown  = gamepad2.right_stick_y;

        //specimen presets
        SpecimenPosOne = gamepad1.right_trigger > 0;
        SpecimenPosTwo = gamepad2.right_trigger > 0;
        SpecimenPosThree = gamepad2.left_trigger > 0;
        PickUpSpecimen = gamepad2.x;
    }
}

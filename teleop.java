/* Copyright (c) 2017 FIRST. All rights reserved.
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
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;


/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="teleoptoggles", group="Pushbot")
public class TELEOP extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot robot           = new HardwarePushbot();   // Use a Pushbot's hardware
    private boolean maskMoveUp = false;
    private boolean maskMoveDown = false;
    final double BUCKET_SPEED = 0.6;

    @Override
    public void runOpMode() {

        //checking teleop distance if encoders convert to inches correctly
        final double COUNTS_PER_MOTOR_REV = 537.7;    // eg: TETRIX Motor Encoder
        final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
        final double WHEEL_DIAMETER_INCHES = 96.0 / 25.4;     // For figuring circumference
        final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);

        double left;
        double right;
        double max;
        double speedControl;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.left_bumper){
                speedControl = 0.3;
            }
            else if(gamepad1.right_bumper){
                speedControl = 0.5;
            }
            else{
                speedControl = 1;
            }

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            double y = -gamepad1.right_stick_y; // Remember, this is reversed!
            double x = gamepad1.right_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.left_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            robot.leftFront.setPower(frontLeftPower * speedControl);
            robot.leftBack.setPower(backLeftPower * speedControl);
            robot.rightFront.setPower(frontRightPower * speedControl);
            robot.rightBack.setPower(backRightPower * speedControl);


            if (gamepad2.dpad_left)
                robot.carousel.setPower(robot.CAROUSEL_POWER_LEFT);
            else if (gamepad2.dpad_right)
                robot.carousel.setPower(robot.CAROUSEL_POWER_RIGHT);
            else
                robot.carousel.setPower(0.0);

            if (gamepad2.y)
                robot.intake.setVelocity(robot.INTAKE_POWER_IN);
            else if (gamepad2.a)
                robot.intake.setVelocity(robot.INTAKE_POWER_OUT);
            else
                robot.intake.setPower(0.0);

            if (gamepad2.right_bumper && !maskMoveUp) {
                maskMoveUp = true;
                robot.moveUp();
            } else if (!gamepad2.right_bumper)
                maskMoveUp = false;

            if (gamepad2.left_bumper && !maskMoveDown) {
                maskMoveDown = true;
                robot.moveDown();
            } else if (!gamepad2.right_bumper)
                maskMoveDown = false;

            if (gamepad2.x) {
                telemetry.addLine("x button pressed");
                robot.bucket.setPosition(HardwarePushbot.BUCKET_OPEN_POSITION);
            }else if (gamepad2.b) {
                telemetry.addLine("b button pressed");
                robot.bucket.setPosition(HardwarePushbot.BUCKET_CLOSE_POSITION);
            }


            telemetry.addData("LF Encoder", robot.leftFront.getCurrentPosition());
            telemetry.addData("LB Encoder", robot.leftBack.getCurrentPosition());
            telemetry.addData("RF Encoder", robot.rightFront.getCurrentPosition());
            telemetry.addData("RB Encoder", robot.rightBack.getCurrentPosition());
            telemetry.addData("LF Inches", robot.leftFront.getCurrentPosition()/COUNTS_PER_INCH);
            telemetry.addData("LB Inches", robot.leftBack.getCurrentPosition()/COUNTS_PER_INCH);
            telemetry.addData("RF Inches", robot.rightFront.getCurrentPosition()/COUNTS_PER_INCH);
            telemetry.addData("RB Inches", robot.rightBack.getCurrentPosition()/COUNTS_PER_INCH);
            telemetry.addData("power:", robot.intake);
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}

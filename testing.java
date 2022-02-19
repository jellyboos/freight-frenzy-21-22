package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//
/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disable led line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name="testing", group="Pushbot")
public class testing extends LinearOpMode {

    /* Declare OpMode members. */
    static final double COUNTS_PER_MOTOR_REV = 537.7;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;

    // Move in a square
    @Override
    public void runOpMode() {
        telemetry.addData("status", "ready");
        telemetry.update();
        robot.init(hardwareMap);

        waitForStart();
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        for (int i = 0; i <= 4; i++) {
            robot.rightFront.setPower(1 - i * 0.2);
            robot.leftFront.setPower(1);
            sleep(1000);
            robot.rightFront.setPower(0);
            robot.leftFront.setPower(0);
            sleep(1000);
        }



//        encoderDrive(DRIVE_SPEED, 24, 0, 0, 0);
//        sleep(1000);
//        encoderDrive(DRIVE_SPEED, 24, 0, 0, 0);
//        sleep(1000);
//        encoderDrive(DRIVE_SPEED, 24, 0, 0, 0);
//        sleep(1000);
//        encoderDrive(DRIVE_SPEED, 24, 0, 0, 0);
//        sleep(1000);

        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();
    }

    // Set motor powers separately
    public void setMotorPowers(double leftFront, double rightFront, double leftBack, double rightBack) {
        robot.leftFront.setPower(leftFront);
        robot.rightFront.setPower(rightFront);
        robot.leftBack.setPower(leftBack);
        robot.rightBack.setPower(rightBack);

    }

    // Set motor powers (same for all 4 motors)
    public void setMotorPowers(double speed) {
        setMotorPowers(speed, speed, speed, speed); //added speed
    }

    // Drive straight at `speed` for `msTime` milliseconds
    public void straight(double speed, int msTime) {
        setMotorPowers(speed);
        idleFor(msTime);
        setMotorPowers(0);
    }

    // Turn right at `turnSpeed` for `msTime` milliseconds
    public void rightTurn(double turnsSpeed, int msTime) {
        setMotorPowers(turnsSpeed, -turnsSpeed, turnsSpeed, -turnsSpeed);//added one more turn speed
        idleFor(msTime);
        setMotorPowers(0);
    }

    private void encoderDrive(double driveSpeed, int i, int i1) {
    }


    //*  Method to perform a relative move, based on encoder counts.
    //*  Encoders are not reset as the move is based on the current position.
    //*  Move will stop if any of three conditions occur:
    //*  1) Move gets to the desired position
    //*  2) Move runs out of time
    // *  3) Driver stops the opmode running.

    // Wait for `msTime` milliseconds
    public void idleFor(int msTime) {
        runtime.reset();
        while (runtime.milliseconds() < msTime) ;
    }

    // *** Turning ***

    // resets currAngle Value
    public void resetAngle() {
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currAngle = 0;
    }

    public double getAngle() {

        // Get current orientation
        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // Change in angle = current angle - previous angle
        double deltaAngle = orientation.firstAngle - lastAngles.firstAngle;

        // Gyro only ranges from -179 to 180
        // If it turns -1 degree over from -179 to 180, subtract 360 from the 359 to get -1
        if (deltaAngle < -180) {
            deltaAngle += 360;
        } else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }

        // Add change in angle to current angle to get current angle
        currAngle += deltaAngle;
        lastAngles = orientation;
        telemetry.addData("gyro", orientation.firstAngle);
        return currAngle;
    }

    public void encoderDrive(double speed, double allMotors) {
        encoderDrive(speed, allMotors, allMotors, allMotors, allMotors);
    }

    public void encoderDrive(double speed, double leftFront, double rightFront, double leftBack, double rightBack) {
        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.leftFront.getCurrentPosition() + (int) (leftFront * COUNTS_PER_INCH);
            newLeftBackTarget = robot.leftBack.getCurrentPosition() + (int) (leftBack * COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightFront.getCurrentPosition() + (int) (rightFront * COUNTS_PER_INCH);
            newRightBackTarget = robot.rightBack.getCurrentPosition() + (int) (rightBack * COUNTS_PER_INCH);
            robot.leftFront.setTargetPosition(newLeftFrontTarget);
            robot.leftBack.setTargetPosition(newLeftBackTarget);
            robot.rightFront.setTargetPosition(newRightFrontTarget);
            robot.rightBack.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftFront.setVelocity(Math.abs(speed) * 312 * COUNTS_PER_MOTOR_REV / 60);
            robot.leftBack.setVelocity(Math.abs(speed) * 312 * COUNTS_PER_MOTOR_REV / 60);
            robot.rightFront.setVelocity(Math.abs(speed) * 312 * COUNTS_PER_MOTOR_REV / 60);
            robot.rightBack.setVelocity(Math.abs(speed) * 312 * COUNTS_PER_MOTOR_REV / 60);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() ||
                    (robot.leftFront.isBusy() || robot.rightFront.isBusy()  || (robot.leftBack.isBusy() || robot.rightBack.isBusy()))) {
            }

            robot.leftFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightBack.setPower(0);
            robot.rightFront.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void turn(double degrees) {
        resetAngle();

        double error = degrees;
//
        while (opModeIsActive() && Math.abs(error) > 2) {
            double motorPower = (error < 0 ? -0.2 : 0.2);
            robot.setMotorPowers(-motorPower, motorPower, -motorPower, motorPower);
            error = degrees - getAngle();
            telemetry.addData("error", error);
            telemetry.addData("currentAngle", getAngle());
            System.out.println(error);
            telemetry.update();
        }

        robot.setMotorPowers(0);
    }

    public void turnTo(double degrees) {

        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double error = degrees - orientation.firstAngle;

        if (error > 180) {
            error -= 360;
        } else if (error < -180) {
            error += 360;
        }

        turn(error);
    }

    public double getAbsoluteAngle() {
        return robot.imu.getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES
        ).firstAngle;
    }

    public void turnPID(double degrees) {
        turnToPID(degrees + getAbsoluteAngle());
    }

    void turnToPID(double targetAngle) {
        TurnPIDController pid = new TurnPIDController(targetAngle, 0.01, 0, 0.003);
        telemetry.setMsTransmissionInterval(50);
        // Checking lastSlope to make sure that it's not oscillating when it quits
        while (Math.abs(targetAngle - getAbsoluteAngle()) > 0.5) {
            double motorPower = pid.update(getAbsoluteAngle());
            robot.setMotorPowers(-motorPower, motorPower, -motorPower, motorPower);

            telemetry.addData("Current Angle", getAbsoluteAngle());
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Slope", pid.getLastSlope());
            telemetry.addData("Power", motorPower);
            telemetry.update();
        }
    }
}

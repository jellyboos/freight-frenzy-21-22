package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;



@Autonomous(name="redtop", group="Pushbot")

public class redtop extends LinearOpMode {

    /* Declare OpMode members. */
    org.firstinspires.ftc.teamcode.HardwarePushbot robot = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    VoltageSensor voltageSensor;
    static final double FORWARD_SPEED = 0.5;
    static final double BACK_SPEED = -0.5 ;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);


        // Wait for the game to start (driver presses PLAY)

        waitForStart();
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        timedMotorPowers(BACK_SPEED,1.08);
        pause();
        runtime.reset();
        outtakeMovement(0.5, 0.5);
        pause();
        robot.bucket.setPosition(0 / 360);
        pause();
        sleep(700);
        robot.bucket.setPosition(200.0 / 360.0);
        pause();
        robot.outtake.setPower(0);
        sleep(1000);
        outtakeMovement(-0.5, 0.53);
        pause();
        robot.outtake.setPower(0);
        sleep(1000);
        timedMotorPowers(0.2, 0.5);
        pause();
        timedMotorPowers(-0.3,0.3,-0.3,0.3,0.5);
        pause();
        timedMotorPowers(0.5, -0.5, -0.5, 0.5, 2.5);
        pause();
        timedMotorPowers(0.25, 1.4);
        pause();
        timedMotorPowers(0.15, 0.5);
        pause();
        robot.carousel.setPower(0.9);
        sleep(3000);
        robot.carousel.setPower(0);
        pause();
        runtime.reset();
        timedMotorPowers(BACK_SPEED,0.75);

    }

    public void timedMotorPowers(double speed , double seconds) {
        robot.setMotorPowers(speed *13/voltageSensor.getVoltage() , speed *13/voltageSensor.getVoltage(), speed *13/voltageSensor.getVoltage(), speed*13/voltageSensor.getVoltage() );
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < seconds)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }
        public void timedMotorPowers (double lf, double rf, double lb, double rb, double seconds) {
            robot.setMotorPowers(lf *13/voltageSensor.getVoltage(), rf *13/voltageSensor.getVoltage(), lb*13/voltageSensor.getVoltage() , rb*13/voltageSensor.getVoltage()); // or however order of motors you do it
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < seconds)) {
                telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
                telemetry.update();
            }
        }
            public void pause(){

            robot.setMotorPowers(0, 0, 0, 0);
            sleep(1000);
            robot.outtake.setPower(0);
            runtime.reset();
        }
        public void outtakeMovement(double power, double seconds){
            robot.outtake.setPower(power);
         runtime.reset();
       while (opModeIsActive() && (runtime.seconds() < seconds)) {
           telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
          telemetry.update();
       }

        }
        }



        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1:  Drive forward for 3 seconds
//        robot.setMotorPowers(BACK_SPEED, BACK_SPEED,  BACK_SPEED, BACK_SPEED);
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 1.1)) {
//
//            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
//            telemetry.update();
//


        // Step 2:  Spin right for 1.3 seconds
//        robot.leftFront.setPower(TURN_SPEED);
//        robot.leftBack.setPower(TURN_SPEED);
//        robot.rightFront.setPower(-TURN_SPEED);
//        robot.rightBack.setPower(-TURN_SPEED);
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 1.3)) {
//            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
//            telemetry.update();



//        // Step 2:  Spin right for 1.3 seconds
//        robot.leftDrive.setPower(TURN_SPEED);
//        robot.rightDrive.setPower(-TURN_SPEED);
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 1.3)) {
//            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }
//
//        // Step 3:  Drive Backwards for 1 Second
//        robot.leftDrive.setPower(-FORWARD_SPEED);
//        robot.rightDrive.setPower(-FORWARD_SPEED);
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
//            telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }
//
//        // Step 4:  Stop and close the claw.
//        robot.leftDrive.setPower(0);
//        robot.rightDrive.setPower(0);
//        robot.leftClaw.setPosition(1.0);
//        robot.rightClaw.setPosition(0.0);
//
//        telemetry.addData("Path", "Complete");
//        telemetry.update();
//        sleep(1000);
//    }
//}

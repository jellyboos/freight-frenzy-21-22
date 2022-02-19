package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwarePushbot {
    /* Public OpMode members. */

    public DcMotorEx leftFront = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx leftBack = null;
    public DcMotorEx rightBack = null;
    public DcMotorEx intake = null;
    public DcMotorEx carousel = null;
    public DcMotorEx outtake = null;
    public Servo bucket = null;
    BNO055IMU imu = null;

    public static final int BOTTOM_OUTTAKE_POSITION = 65;
    public static final int MID_OUTTAKE_POSITION = 1000;
    public static final int TOP_OUTTAKE_POSITION = 1600;
    public static final double OUTTAKE_SPEED = 0.3 * 117 * 1425.1 / 60;
    public static final double CAROUSEL_POWER_LEFT = 1;
    public static final double CAROUSEL_POWER_RIGHT = -1;
    public static final double INTAKE_POWER_IN  = 0.65 * 1150 * 145.1 / 60;
    public static final double INTAKE_POWER_OUT = -0.65 * 1150 * 145.1 / 60;

    public static final double BUCKET_OPEN_POSITION = 200.0/360.0;
    public static final double BUCKET_CLOSE_POSITION = 15/360.0;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwarePushbot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFront = hwMap.get(DcMotorEx.class, "LF");
        rightFront = hwMap.get(DcMotorEx.class, "RF");
        leftBack = hwMap.get(DcMotorEx.class, "LB");
        rightBack = hwMap.get(DcMotorEx.class, "RB");
        intake = hwMap.get(DcMotorEx.class, "intake");
        carousel = hwMap.get(DcMotorEx.class, "C");
        outtake = hwMap.get(DcMotorEx.class, "outtake");
        imu = hwMap.get(BNO055IMU.class, "imu");

        bucket  = hwMap.get(Servo.class, "bucket");
        bucket.setPosition(200.0/360);

        // Set all motors to zero power
        setMotorPowers(0);
        intake.setPower(0);
        carousel.setPower(0);
        outtake.setPower(0);

        //reverse wheels
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);



        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        carousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Define and initialize ALL installed servos.
    }
    public void setMotorPowers(double p){
        setMotorPowers(p,p,p,p);
    }

    public void setMotorPowers(double lF, double rF, double lB, double rB) {
        leftFront.setPower(lF);
        leftBack.setPower(lB);
        rightBack.setPower(rB);
        rightFront.setPower(rF);
    }

    public void moveUp() {
        int targetPosition = 0;
        if (outtake.getCurrentPosition() < MID_OUTTAKE_POSITION)
            targetPosition = MID_OUTTAKE_POSITION;
        else if (outtake.getCurrentPosition() < TOP_OUTTAKE_POSITION)
            targetPosition = TOP_OUTTAKE_POSITION;
        outtake.setTargetPosition(targetPosition);
        outtake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtake.setVelocity(OUTTAKE_SPEED);
    }

    public void moveDown() {
        int targetPosition = 0;
        if (outtake.getCurrentPosition() >= MID_OUTTAKE_POSITION)
            targetPosition = BOTTOM_OUTTAKE_POSITION;
        outtake.setTargetPosition(targetPosition);
        outtake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtake.setVelocity(OUTTAKE_SPEED);
   }
 }

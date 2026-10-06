package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

@Autonomous(name = "TEST SOL AUTO", group = "Autonomous")
public class TEST_SOL_AUTO extends LinearOpMode {

    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;

    private Limelight3A limelight;
    private SparkFunOTOS otos;

    private static final double SQUARE_SIZE = 48.0;

    /*
     * 537.7 is common for goBILDA 312 RPM motors.
     * If your motors use a different encoder count,
     * this value will need to be changed.
     */
    private static final double TICKS_PER_REV = 537.7;
    private static final double WHEEL_DIAMETER = 4.0;

    private static final double TICKS_PER_INCH =
            TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER);

    private static final double DRIVE_POWER = 0.55;

    private static final double TAG_STRAFE_POWER = 0.35;
    private static final long TAG_STRAFE_TIME = 1000;

    @Override
    public void runOpMode() throws InterruptedException {

        initializeMotors();
        initializeLimelight();
        initializeOTOS();

        telemetry.addLine("Initialized");
        telemetry.addData(
                "Limelight",
                limelight != null ? "Connected" : "Not found"
        );
        telemetry.addData(
                "OTOS",
                otos != null ? "Connected" : "Not found"
        );
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            stopRobot();
            return;
        }

        /*
         * 4 ft x 4 ft square
         *
         * 1. Left 48"
         * 2. Forward 48"
         * 3. Right 48"
         * 4. Backward 48"
         */

        strafeLeft(SQUARE_SIZE);

        if (!opModeIsActive()) {
            stopRobot();
            return;
        }

        driveForward(SQUARE_SIZE);

        if (!opModeIsActive()) {
            stopRobot();
            return;
        }

        strafeRight(SQUARE_SIZE);

        if (!opModeIsActive()) {
            stopRobot();
            return;
        }

        driveBackward(SQUARE_SIZE);

        stopRobot();

        /*
         * The square is completely finished here.
         * AprilTags are not checked before this point.
         */
        sleep(500);

        aprilTagAction();

        stopRobot();
    }

    private void initializeMotors() {

        frontLeftDrive =
                hardwareMap.get(
                        DcMotor.class,
                        "front_left_drive"
                );

        frontRightDrive =
                hardwareMap.get(
                        DcMotor.class,
                        "front_right_drive"
                );

        backLeftDrive =
                hardwareMap.get(
                        DcMotor.class,
                        "back_left_drive"
                );

        backRightDrive =
                hardwareMap.get(
                        DcMotor.class,
                        "back_right_drive"
                );

        /*
         * This matches the direction setup used
         * in the official FTC mecanum sample.
         */
        frontLeftDrive.setDirection(
                DcMotor.Direction.REVERSE
        );

        backLeftDrive.setDirection(
                DcMotor.Direction.REVERSE
        );

        frontRightDrive.setDirection(
                DcMotor.Direction.FORWARD
        );

        backRightDrive.setDirection(
                DcMotor.Direction.FORWARD
        );

        resetEncoders();

        setRunMode(
                DcMotor.RunMode.RUN_USING_ENCODER
        );

        stopRobot();
    }

    private void initializeLimelight() {

        try {

            limelight =
                    hardwareMap.get(
                            Limelight3A.class,
                            "limelight"
                    );

            limelight.pipelineSwitch(0);
            limelight.start();

        } catch (Exception e) {

            limelight = null;
        }
    }

    private void initializeOTOS() {

        try {

            otos =
                    hardwareMap.get(
                            SparkFunOTOS.class,
                            "sensor_otos"
                    );

            otos.setLinearUnit(
                    DistanceUnit.INCH
            );

            otos.setAngularUnit(
                    AngleUnit.DEGREES
            );

            otos.resetTracking();

            otos.setPosition(
                    new SparkFunOTOS.Pose2D(
                            0,
                            0,
                            0
                    )
            );

        } catch (Exception e) {

            otos = null;
        }
    }

    private void resetEncoders() {

        frontLeftDrive.setMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER
        );

        frontRightDrive.setMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER
        );

        backLeftDrive.setMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER
        );

        backRightDrive.setMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER
        );
    }

    private void setRunMode(DcMotor.RunMode mode) {

        frontLeftDrive.setMode(mode);
        frontRightDrive.setMode(mode);
        backLeftDrive.setMode(mode);
        backRightDrive.setMode(mode);
    }

    private void driveForward(double inches) {

        moveRobot(
                inches,
                inches,
                inches,
                inches,
                "FORWARD"
        );
    }

    private void driveBackward(double inches) {

        moveRobot(
                -inches,
                -inches,
                -inches,
                -inches,
                "BACKWARD"
        );
    }

    private void strafeLeft(double inches) {

        /*
         * Official mecanum convention:
         *
         * FL = -
         * FR = +
         * BL = +
         * BR = -
         */
        moveRobot(
                -inches,
                inches,
                inches,
                -inches,
                "STRAFE LEFT"
        );
    }

    private void strafeRight(double inches) {

        /*
         * Opposite of strafe left.
         */
        moveRobot(
                inches,
                -inches,
                -inches,
                inches,
                "STRAFE RIGHT"
        );
    }

    private void moveRobot(
            double frontLeftInches,
            double frontRightInches,
            double backLeftInches,
            double backRightInches,
            String movement) {

        int frontLeftTarget =
                (int) Math.round(
                        frontLeftInches * TICKS_PER_INCH
                );

        int frontRightTarget =
                (int) Math.round(
                        frontRightInches * TICKS_PER_INCH
                );

        int backLeftTarget =
                (int) Math.round(
                        backLeftInches * TICKS_PER_INCH
                );

        int backRightTarget =
                (int) Math.round(
                        backRightInches * TICKS_PER_INCH
                );

        /*
         * IMPORTANT:
         *
         * Set every target BEFORE changing
         * any motor to RUN_TO_POSITION.
         */
        resetEncoders();

        frontLeftDrive.setTargetPosition(
                frontLeftTarget
        );

        frontRightDrive.setTargetPosition(
                frontRightTarget
        );

        backLeftDrive.setTargetPosition(
                backLeftTarget
        );

        backRightDrive.setTargetPosition(
                backRightTarget
        );

        setRunMode(
                DcMotor.RunMode.RUN_TO_POSITION
        );

        frontLeftDrive.setPower(DRIVE_POWER);
        frontRightDrive.setPower(DRIVE_POWER);
        backLeftDrive.setPower(DRIVE_POWER);
        backRightDrive.setPower(DRIVE_POWER);

        while (
                opModeIsActive() &&
                        (
                                frontLeftDrive.isBusy() ||
                                        frontRightDrive.isBusy() ||
                                        backLeftDrive.isBusy() ||
                                        backRightDrive.isBusy()
                        )
        ) {

            telemetry.addData(
                    "Movement",
                    movement
            );

            telemetry.addData(
                    "FL",
                    "%d / %d",
                    frontLeftDrive.getCurrentPosition(),
                    frontLeftTarget
            );

            telemetry.addData(
                    "FR",
                    "%d / %d",
                    frontRightDrive.getCurrentPosition(),
                    frontRightTarget
            );

            telemetry.addData(
                    "BL",
                    "%d / %d",
                    backLeftDrive.getCurrentPosition(),
                    backLeftTarget
            );

            telemetry.addData(
                    "BR",
                    "%d / %d",
                    backRightDrive.getCurrentPosition(),
                    backRightTarget
            );

            telemetry.update();
        }

        stopRobot();

        setRunMode(
                DcMotor.RunMode.RUN_USING_ENCODER
        );

        sleep(200);
    }

    private void aprilTagAction()
            throws InterruptedException {

        if (limelight == null) {
            telemetry.addLine(
                    "Limelight unavailable."
            );
            telemetry.update();
            return;
        }

        long startTime =
                System.currentTimeMillis();

        while (
                opModeIsActive() &&
                        System.currentTimeMillis() - startTime < 5000
        ) {

            LLResult result =
                    limelight.getLatestResult();

            if (result != null &&
                    result.isValid()) {

                List<LLResultTypes.FiducialResult>
                        tags =
                        result.getFiducialResults();

                for (
                        LLResultTypes.FiducialResult tag :
                        tags
                ) {

                    int id =
                            tag.getFiducialId();

                    telemetry.addData(
                            "AprilTag",
                            id
                    );

                    telemetry.update();

                    if (id == 42 || id == 43) {

                        strafeLeftForTime();
                        return;
                    }

                    if (id == 44 || id == 45) {

                        strafeRightForTime();
                        return;
                    }
                }
            }

            stopRobot();
            sleep(20);
        }

        stopRobot();
    }

    private void strafeLeftForTime()
            throws InterruptedException {

        long start =
                System.currentTimeMillis();

        while (
                opModeIsActive() &&
                        System.currentTimeMillis() - start <
                                TAG_STRAFE_TIME
        ) {

            /*
             * Same mecanum pattern as the
             * official FTC strafe-left convention.
             */
            setMecanumPower(
                    -TAG_STRAFE_POWER,
                    TAG_STRAFE_POWER,
                    TAG_STRAFE_POWER,
                    -TAG_STRAFE_POWER
            );

            telemetry.addLine(
                    "ID 42/43 -> STRAFE LEFT"
            );

            telemetry.update();
        }

        stopRobot();
    }

    private void strafeRightForTime()
            throws InterruptedException {

        long start =
                System.currentTimeMillis();

        while (
                opModeIsActive() &&
                        System.currentTimeMillis() - start <
                                TAG_STRAFE_TIME
        ) {

            setMecanumPower(
                    TAG_STRAFE_POWER,
                    -TAG_STRAFE_POWER,
                    -TAG_STRAFE_POWER,
                    TAG_STRAFE_POWER
            );

            telemetry.addLine(
                    "ID 44/45 -> STRAFE RIGHT"
            );

            telemetry.update();
        }

        stopRobot();
    }

    private void setMecanumPower(
            double frontLeft,
            double frontRight,
            double backLeft,
            double backRight) {

        frontLeftDrive.setPower(frontLeft);
        frontRightDrive.setPower(frontRight);
        backLeftDrive.setPower(backLeft);
        backRightDrive.setPower(backRight);
    }

    private void stopRobot() {

        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }
}
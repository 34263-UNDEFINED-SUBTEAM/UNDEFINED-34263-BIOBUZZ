
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "TEST SOL AUTO", group = "Autonomous")
public class TEST_SOL_AUTO extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // Robot dimensions and encoder settings.
    private static final double WHEEL_DIAMETER_INCHES = 4.0;
    private static final double WHEEL_CIRCUMFERENCE =
            Math.PI * WHEEL_DIAMETER_INCHES;

    // Starting estimate for a goBILDA 5202 19.2:1 motor.
    // Verify this value for your exact motor.
    private static final double TICKS_PER_MOTOR_REV = 537.7;

    // Change this if your motor's actual gear ratio differs.
    private static final double DRIVE_GEAR_RATIO = 1.0;

    // Adjust after testing; mecanum strafing typically needs calibration.
    private static final double STRAFE_FACTOR = 1.15;

    private static final double SIDE_LENGTH_INCHES = 48.0;
    private static final double DRIVE_POWER = 0.35;
    private static final double STRAFE_POWER = 0.30;
    private static final double MOVE_TIMEOUT_SECONDS = 8.0;

    @Override
    public void runOpMode() {

        // Hardware names must match Robot Configuration.
        frontLeft = hardwareMap.get(
                DcMotor.class, "front_left_drive");
        frontRight = hardwareMap.get(
                DcMotor.class, "front_right_drive");
        backLeft = hardwareMap.get(
                DcMotor.class, "back_left_drive");
        backRight = hardwareMap.get(
                DcMotor.class, "back_right_drive");

        // Standard mecanum motor directions.
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // Brake when power is zero.
        frontLeft.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        resetDriveEncoders();

        telemetry.addLine("TEST_SOL_AUTO");
        telemetry.addLine("Encoder-only 48-inch square");
        telemetry.addData("Wheel diameter", "%.1f in",
                WHEEL_DIAMETER_INCHES);
        telemetry.addData("Ticks per motor revolution", "%.1f",
                TICKS_PER_MOTOR_REV);
        telemetry.addLine("Verify gear ratio before running.");
        telemetry.addLine("Place robot in an open test area.");
        telemetry.update();

        waitForStart();

        if (!opModeIsActive()) {
            stopDrive();
            return;
        }

        // Side 1: forward 48 inches.
        if (!driveStraight(SIDE_LENGTH_INCHES, DRIVE_POWER)) {
            stopDrive();
            return;
        }

        // Side 2: strafe right 48 inches.
        if (!strafe(SIDE_LENGTH_INCHES, STRAFE_POWER)) {
            stopDrive();
            return;
        }

        // Side 3: backward 48 inches.
        if (!driveStraight(-SIDE_LENGTH_INCHES, DRIVE_POWER)) {
            stopDrive();
            return;
        }

        // Side 4: strafe left 48 inches.
        if (!strafe(-SIDE_LENGTH_INCHES, STRAFE_POWER)) {
            stopDrive();
            return;
        }

        stopDrive();

        telemetry.addLine("SQUARE COMPLETE");
        telemetry.addLine("Encoder-based movement finished.");
        telemetry.update();
    }

    private boolean driveStraight(double inches, double power) {

        int ticks = inchesToTicks(inches);

        setTargetPositions(
                ticks, ticks, ticks, ticks);

        return runToPosition(
                power, power, power, power,
                "Driving straight");
    }

    private boolean strafe(double inches, double power) {

        int ticks = inchesToTicks(
                inches * STRAFE_FACTOR);

        // Mecanum strafe pattern.
        setTargetPositions(
                ticks, -ticks, -ticks, ticks);

        return runToPosition(
                power, power, power, power,
                inches > 0 ? "Strafing right" : "Strafing left");
    }

    private int inchesToTicks(double inches) {

        double wheelRevolutions =
                inches / WHEEL_CIRCUMFERENCE;

        double motorRevolutions =
                wheelRevolutions * DRIVE_GEAR_RATIO;

        return (int) Math.round(
                motorRevolutions * TICKS_PER_MOTOR_REV);
    }

    private void setTargetPositions(
            int fl, int fr, int bl, int br) {

        frontLeft.setTargetPosition(
                frontLeft.getCurrentPosition() + fl);
        frontRight.setTargetPosition(
                frontRight.getCurrentPosition() + fr);
        backLeft.setTargetPosition(
                backLeft.getCurrentPosition() + bl);
        backRight.setTargetPosition(
                backRight.getCurrentPosition() + br);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private boolean runToPosition(
            double flPower, double frPower,
            double blPower, double brPower,
            String movementName) {

        frontLeft.setPower(Math.abs(flPower));
        frontRight.setPower(Math.abs(frPower));
        backLeft.setPower(Math.abs(blPower));
        backRight.setPower(Math.abs(brPower));

        long startTime = System.nanoTime();

        while (opModeIsActive()
                && anyMotorBusy()
                && (System.nanoTime() - startTime)
                / 1_000_000_000.0 < MOVE_TIMEOUT_SECONDS) {

            telemetry.addData("Movement", movementName);
            telemetry.addData("Front Left", "%d / %d",
                    frontLeft.getCurrentPosition(),
                    frontLeft.getTargetPosition());
            telemetry.addData("Front Right", "%d / %d",
                    frontRight.getCurrentPosition(),
                    frontRight.getTargetPosition());
            telemetry.addData("Back Left", "%d / %d",
                    backLeft.getCurrentPosition(),
                    backLeft.getTargetPosition());
            telemetry.addData("Back Right", "%d / %d",
                    backRight.getCurrentPosition(),
                    backRight.getTargetPosition());

            telemetry.update();
            idle();
        }

        boolean completed = opModeIsActive()
                && !anyMotorBusy();

        stopDrive();

        if (!completed && opModeIsActive()) {
            telemetry.addLine(
                    "TIMEOUT: Movement did not finish.");
            telemetry.update();
        }

        // Return to open-loop mode for the next movement.
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        return completed;
    }

    private boolean anyMotorBusy() {
        return frontLeft.isBusy()
                || frontRight.isBusy()
                || backLeft.isBusy()
                || backRight.isBusy();
    }

    private void resetDriveEncoders() {

        setDrivePower(0);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setDriveMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    private void setDrivePower(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    private void stopDrive() {
        setDrivePower(0);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}


package org.firstinspires.ftc.teamcode.Code_Backups;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "DRIVE INTAKE TEST", group = "Test")
public class DRIVE_INTAKE_TEST extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private Servo leftIntake;

    private static final double WHEEL_DIAMETER = 4.0;
    private static final double TICKS_PER_REV = 537.7;
    private static final double STRAFE_FACTOR = 1.15;

    private static final double TEST_DISTANCE = 12.0;
    private static final double DRIVE_POWER = 0.25;
    private static final double STRAFE_POWER = 0.20;
    private static final double MOVE_TIMEOUT = 8.0;

    private static final double INTAKE_OPEN = 0.20;
    private static final double INTAKE_CLOSED = 0.80;

    @Override
    public void runOpMode() {

        // Hardware initialization
        frontLeft = hardwareMap.get(
                DcMotor.class, "front_left_drive");

        frontRight = hardwareMap.get(
                DcMotor.class, "front_right_drive");

        backLeft = hardwareMap.get(
                DcMotor.class, "back_left_drive");

        backRight = hardwareMap.get(
                DcMotor.class, "back_right_drive");

        // IMPORTANT: This must exactly match Robot Configuration.
        leftIntake = hardwareMap.get(
                Servo.class, "Left servo intake");

        // Motor directions
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        setBrake();
        stopDrive();
        resetEncoders();

        leftIntake.setPosition(INTAKE_OPEN);

        telemetry.addLine("DRIVE INTAKE TEST");
        telemetry.addLine("Ready to start.");
        telemetry.addData("Servo name", "Left servo intake");
        telemetry.update();

        waitForStart();

        if (!opModeIsActive()) {
            finish();
            return;
        }

        // 1. BACKWARD AND RETURN
        if (!drive(-TEST_DISTANCE)) return;
        if (!drive(TEST_DISTANCE)) return;

        // 2. FORWARD AND RETURN
        if (!drive(TEST_DISTANCE)) return;
        if (!drive(-TEST_DISTANCE)) return;

        // 3. LEFT AND RETURN
        if (!strafe(-TEST_DISTANCE)) return;
        if (!strafe(TEST_DISTANCE)) return;

        // 4. RIGHT AND RETURN
        if (!strafe(TEST_DISTANCE)) return;
        if (!strafe(-TEST_DISTANCE)) return;

        stopDrive();

        // 5. INTAKE SERVO TEST
        telemetry.addLine("Testing intake servo...");
        telemetry.update();

        leftIntake.setPosition(INTAKE_OPEN);
        sleep(1000);

        if (!opModeIsActive()) {
            finish();
            return;
        }

        leftIntake.setPosition(INTAKE_CLOSED);
        sleep(1500);

        if (!opModeIsActive()) {
            finish();
            return;
        }

        leftIntake.setPosition(INTAKE_OPEN);
        sleep(1000);

        finish();

        telemetry.addLine("ALL TESTS COMPLETE");
        telemetry.update();
    }

    private boolean drive(double inches) {

        int ticks = inchesToTicks(inches);

        setTargets(ticks, ticks, ticks, ticks);

        return runMovement(
                DRIVE_POWER,
                inches < 0 ? "Backward" : "Forward");
    }

    private boolean strafe(double inches) {

        int ticks = inchesToTicks(
                inches * STRAFE_FACTOR);

        setTargets(
                ticks,
                -ticks,
                -ticks,
                ticks);

        return runMovement(
                STRAFE_POWER,
                inches < 0 ? "Left" : "Right");
    }

    private int inchesToTicks(double inches) {

        double circumference = Math.PI * WHEEL_DIAMETER;

        return (int) Math.round(
                inches / circumference * TICKS_PER_REV);
    }

    private void setTargets(
            int fl, int fr, int bl, int br) {

        frontLeft.setTargetPosition(
                frontLeft.getCurrentPosition() + fl);

        frontRight.setTargetPosition(
                frontRight.getCurrentPosition() + fr);

        backLeft.setTargetPosition(
                backLeft.getCurrentPosition() + bl);

        backRight.setTargetPosition(
                backRight.getCurrentPosition() + br);

        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private boolean runMovement(
            double power, String movement) {

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()
                && anyMotorBusy()
                && timer.seconds() < MOVE_TIMEOUT) {

            telemetry.addData("Movement", movement);
            telemetry.addData("Time", "%.1f",
                    timer.seconds());

            telemetry.addData("FL",
                    "%d / %d",
                    frontLeft.getCurrentPosition(),
                    frontLeft.getTargetPosition());

            telemetry.addData("FR",
                    "%d / %d",
                    frontRight.getCurrentPosition(),
                    frontRight.getTargetPosition());

            telemetry.addData("BL",
                    "%d / %d",
                    backLeft.getCurrentPosition(),
                    backLeft.getTargetPosition());

            telemetry.addData("BR",
                    "%d / %d",
                    backRight.getCurrentPosition(),
                    backRight.getTargetPosition());

            telemetry.update();
            idle();
        }

        boolean completed =
                opModeIsActive() && !anyMotorBusy();

        stopDrive();
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (!completed && opModeIsActive()) {
            telemetry.addLine("MOVEMENT TIMED OUT");
            telemetry.update();
        }

        return completed;
    }

    private boolean anyMotorBusy() {

        return frontLeft.isBusy()
                || frontRight.isBusy()
                || backLeft.isBusy()
                || backRight.isBusy();
    }

    private void resetEncoders() {

        setDriveMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setDriveMode(
                DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setDriveMode(DcMotor.RunMode mode) {

        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    private void setBrake() {

        frontLeft.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        backLeft.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        backRight.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void stopDrive() {

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private void finish() {

        stopDrive();
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}

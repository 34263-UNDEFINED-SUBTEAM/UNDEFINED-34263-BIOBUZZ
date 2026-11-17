
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "PA - Pollen Auto", group = "Autonomous")
public class PA extends LinearOpMode {

    // Drive motors
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private DcMotor[] motors;

    // Tested robot calibration
    private static final double TICKS_PER_INCH = 41.81;
    private static final double TURN_DIAMETER_INCHES = 28.0;

    // Movement settings
    private static final double DRIVE_POWER = 0.55;
    private static final double TURN_POWER = 0.45;

    // Autonomous safety
    private static final double MATCH_LIMIT_SECONDS = 29.0;

    private final ElapsedTime matchTimer = new ElapsedTime();

    @Override
    public void runOpMode() {

        telemetry.addLine("PA - Pollen Auto");
        telemetry.addLine("Initializing hardware...");
        telemetry.update();

        if (!initializeHardware()) {
            telemetry.addLine("Hardware initialization failed!");
            telemetry.update();
            return;
        }

        telemetry.addLine("Hardware initialized.");
        telemetry.addLine("Starting position: Ready");
        telemetry.addLine("Waiting for START...");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            stopDrive();
            return;
        }

        matchTimer.reset();

        telemetry.addLine("PA STARTED");
        telemetry.update();

        boolean completed = runPollenRoute();

        stopDrive();

        telemetry.addLine(completed
                ? "PA ROUTE COMPLETED"
                : "PA STOPPED - TIMEOUT OR STOP REQUEST");

        telemetry.addData("Elapsed time", "%.2f seconds",
                matchTimer.seconds());

        telemetry.update();
    }

    private boolean initializeHardware() {

        try {
            frontLeft = hardwareMap.get(
                    DcMotor.class, "front_left_drive");

            frontRight = hardwareMap.get(
                    DcMotor.class, "front_right_drive");

            backLeft = hardwareMap.get(
                    DcMotor.class, "back_left_drive");

            backRight = hardwareMap.get(
                    DcMotor.class, "back_right_drive");

        } catch (Exception e) {
            telemetry.addData("Hardware error", e.getMessage());
            telemetry.update();
            return false;
        }

        motors = new DcMotor[]{
                frontLeft,
                frontRight,
                backLeft,
                backRight
        };

        // Preserve tested motor directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(
                    DcMotor.ZeroPowerBehavior.BRAKE);

            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setPower(0);
        }

        return true;
    }

    // Pollen Auto route
    private boolean runPollenRoute() {

        // 1. Forward 3 feet
        if (!drive(36, 0)) return false;

        // 2. Turn 90 degrees left
        if (!turn(-90)) return false;

        // 3. Turn 225 degrees right
        if (!turn(225)) return false;

        // 4. Forward 7 feet
        if (!drive(84, 0)) return false;

        // 5. Turn 180 degrees right
        if (!turn(180)) return false;

        // 6. Forward 7 feet
        if (!drive(84, 0)) return false;

        // 7. Back up 1 foot
        if (!drive(-12, 0)) return false;

        // 8. Turn 125 degrees right
        if (!turn(125)) return false;

        // 9. Forward 9 feet
        if (!drive(108, 0)) return false;

        return true;
    }

    // Encoder-based mecanum movement
    // Positive forward = forward
    // Negative forward = backward
    // Positive strafeRight = right
    // Negative strafeRight = left
    private boolean drive(double forward, double strafeRight) {

        if (!canContinue()) return false;

        int flTicks = (int) Math.round(
                (forward + strafeRight) * TICKS_PER_INCH);

        int frTicks = (int) Math.round(
                (forward - strafeRight) * TICKS_PER_INCH);

        int blTicks = (int) Math.round(
                (forward - strafeRight) * TICKS_PER_INCH);

        int brTicks = (int) Math.round(
                (forward + strafeRight) * TICKS_PER_INCH);

        int[] targets = {
                frontLeft.getCurrentPosition() + flTicks,
                frontRight.getCurrentPosition() + frTicks,
                backLeft.getCurrentPosition() + blTicks,
                backRight.getCurrentPosition() + brTicks
        };

        double maxDistance = Math.max(
                Math.abs(forward + strafeRight),
                Math.abs(forward - strafeRight));

        double timeout = 2.0 + maxDistance / 18.0;

        telemetry.addData("Movement", "Drive");
        telemetry.addData("Forward", "%.1f in", forward);
        telemetry.addData("Strafe right", "%.1f in", strafeRight);
        telemetry.update();

        return runToPosition(targets, DRIVE_POWER, timeout);
    }

    // Positive degrees = clockwise/right
    // Negative degrees = counterclockwise/left
    private boolean turn(double degrees) {

        if (!canContinue()) return false;

        double wheelDistance =
                Math.PI * TURN_DIAMETER_INCHES
                        * Math.abs(degrees) / 360.0;

        int ticks = (int) Math.round(
                wheelDistance * TICKS_PER_INCH);

        int direction = degrees >= 0 ? 1 : -1;

        int[] targets = {
                frontLeft.getCurrentPosition() + direction * ticks,
                frontRight.getCurrentPosition() - direction * ticks,
                backLeft.getCurrentPosition() + direction * ticks,
                backRight.getCurrentPosition() - direction * ticks
        };

        double timeout = 2.0 + wheelDistance / 18.0;

        telemetry.addData("Movement", "Turn");
        telemetry.addData("Degrees", "%.1f", degrees);
        telemetry.addData("Direction",
                degrees >= 0 ? "Right / Clockwise" : "Left / Counterclockwise");
        telemetry.update();

        return runToPosition(targets, TURN_POWER, timeout);
    }

    // Shared encoder movement controller
    private boolean runToPosition(
            int[] targets,
            double power,
            double timeoutSeconds) {

        if (!canContinue()) return false;

        for (int i = 0; i < motors.length; i++) {
            motors[i].setTargetPosition(targets[i]);
            motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motors[i].setPower(Math.abs(power));
        }

        ElapsedTime movementTimer = new ElapsedTime();

        while (opModeIsActive()
                && movementTimer.seconds() < timeoutSeconds
                && matchTimer.seconds() < MATCH_LIMIT_SECONDS
                && anyMotorBusy()) {

            telemetry.addData("PA time", "%.1f / %.1f",
                    matchTimer.seconds(), MATCH_LIMIT_SECONDS);

            telemetry.addData("Movement time", "%.1f",
                    movementTimer.seconds());

            telemetry.addData("FL", "%d / %d",
                    frontLeft.getCurrentPosition(),
                    frontLeft.getTargetPosition());

            telemetry.addData("FR", "%d / %d",
                    frontRight.getCurrentPosition(),
                    frontRight.getTargetPosition());

            telemetry.addData("BL", "%d / %d",
                    backLeft.getCurrentPosition(),
                    backLeft.getTargetPosition());

            telemetry.addData("BR", "%d / %d",
                    backRight.getCurrentPosition(),
                    backRight.getTargetPosition());

            telemetry.update();
            idle();
        }

        boolean successful = opModeIsActive()
                && movementTimer.seconds() < timeoutSeconds
                && matchTimer.seconds() < MATCH_LIMIT_SECONDS
                && !anyMotorBusy();

        stopDrive();

        return successful;
    }

    private boolean anyMotorBusy() {
        for (DcMotor motor : motors) {
            if (motor.isBusy()) {
                return true;
            }
        }
        return false;
    }

    private boolean canContinue() {
        return opModeIsActive()
                && matchTimer.seconds() < MATCH_LIMIT_SECONDS;
    }

    private void stopDrive() {

        if (motors == null) return;

        for (DcMotor motor : motors) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}

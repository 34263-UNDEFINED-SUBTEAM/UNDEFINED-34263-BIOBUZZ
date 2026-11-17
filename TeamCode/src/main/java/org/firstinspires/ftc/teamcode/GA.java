
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "GA - Garden Auto", group = "Autonomous")
public class GA extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // Existing robot calibration
    private static final double TICKS_PER_INCH = 41.81;
    private static final double TURN_DIAMETER_INCHES = 28.0;

    private static final double DRIVE_POWER = 0.55;
    private static final double TURN_POWER = 0.45;

    private static final double MATCH_LIMIT_SECONDS = 29.0;

    private final ElapsedTime matchTimer = new ElapsedTime();

    @Override
    public void runOpMode() {

        initializeHardware();

        telemetry.addLine("GA - Garden Auto");
        telemetry.addLine("Hardware initialized");
        telemetry.addData("Ticks per inch", TICKS_PER_INCH);
        telemetry.addData("Turn diameter", TURN_DIAMETER_INCHES);
        telemetry.addLine("Ready to start");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            stopDrive();
            return;
        }

        matchTimer.reset();

        boolean completed = runGardenRoute();

        stopDrive();

        telemetry.addLine(completed
                ? "GARDEN AUTO COMPLETED"
                : "GARDEN AUTO STOPPED");

        telemetry.addData("Elapsed time", "%.1f seconds",
                matchTimer.seconds());
        telemetry.update();

        sleep(1000);
    }

    private void initializeHardware() {

        frontLeft = hardwareMap.get(
                DcMotor.class, "front_left_drive");
        frontRight = hardwareMap.get(
                DcMotor.class, "front_right_drive");
        backLeft = hardwareMap.get(
                DcMotor.class, "back_left_drive");
        backRight = hardwareMap.get(
                DcMotor.class, "back_right_drive");

        // Tested motor directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        DcMotor[] motors = {
                frontLeft, frontRight, backLeft, backRight
        };

        for (DcMotor motor : motors) {
            motor.setPower(0);
            motor.setZeroPowerBehavior(
                    DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    // GARDEN AUTO ROUTE
    private boolean runGardenRoute() {

        // 1. Forward 4 feet
        if (!runStep("1: Forward 4 feet", () ->
                drive(48, 0))) return false;

        // 2. Turn 90 degrees right
        if (!runStep("2: Turn 90 right", () ->
                turn(90))) return false;

        // 3. Backward 1 foot
        if (!runStep("3: Backward 1 foot", () ->
                drive(-12, 0))) return false;

        // 4. Wait 2 seconds
        if (!runStep("4: Wait 2 seconds", () ->
                waitSafely(2000))) return false;

        // 5. Turn 180 degrees right
        if (!runStep("5: Turn 180 right", () ->
                turn(180))) return false;

        // 6. Forward 2 feet
        if (!runStep("6: Forward 2 feet", () ->
                drive(24, 0))) return false;

        // 7. Backward 2 feet
        if (!runStep("7: Backward 2 feet", () ->
                drive(-24, 0))) return false;

        // 8. Turn 90 degrees right
        if (!runStep("8: Turn 90 right", () ->
                turn(90))) return false;

        // 9. Forward 1.9 feet
        if (!runStep("9: Forward 1.9 feet", () ->
                drive(22.8, 0))) return false;

        return true;
    }

    private interface Movement {
        boolean execute();
    }

    private boolean runStep(String step, Movement movement) {

        if (!canContinue()) return false;

        telemetry.addLine(step);
        telemetry.addData("Match time", "%.1f seconds",
                matchTimer.seconds());
        telemetry.update();

        boolean result = movement.execute();

        stopDrive();

        telemetry.addData("Last step", step);
        telemetry.addData("Result", result ? "Completed" : "Failed");
        telemetry.update();

        return result && canContinue();
    }

    // Positive forward = forward
    // Positive strafeRight = right
    private boolean drive(double forwardInches,
                          double strafeRightInches) {

        if (!canContinue()) return false;

        int flTicks = (int) Math.round(
                (forwardInches + strafeRightInches) * TICKS_PER_INCH);

        int frTicks = (int) Math.round(
                (forwardInches - strafeRightInches) * TICKS_PER_INCH);

        int blTicks = (int) Math.round(
                (forwardInches - strafeRightInches) * TICKS_PER_INCH);

        int brTicks = (int) Math.round(
                (forwardInches + strafeRightInches) * TICKS_PER_INCH);

        return runToPosition(
                flTicks, frTicks, blTicks, brTicks,
                DRIVE_POWER,
                Math.abs(forwardInches) + Math.abs(strafeRightInches)
        );
    }

    // Positive degrees = right / clockwise
    // Negative degrees = left / counterclockwise
    private boolean turn(double degrees) {

        if (!canContinue()) return false;

        double wheelDistance =
                Math.PI * TURN_DIAMETER_INCHES
                        * Math.abs(degrees) / 360.0;

        int ticks = (int) Math.round(
                wheelDistance * TICKS_PER_INCH);

        int direction = degrees >= 0 ? 1 : -1;

        int flTicks = direction * ticks;
        int frTicks = -direction * ticks;
        int blTicks = direction * ticks;
        int brTicks = -direction * ticks;

        telemetry.addData("Turn command", "%.1f degrees", degrees);
        telemetry.addData("Turn encoder ticks", ticks);
        telemetry.update();

        return runToPosition(
                flTicks, frTicks, blTicks, brTicks,
                TURN_POWER,
                wheelDistance
        );
    }

    private boolean runToPosition(
            int flTicks,
            int frTicks,
            int blTicks,
            int brTicks,
            double power,
            double distanceInches) {

        if (!canContinue()) return false;

        DcMotor[] motors = {
                frontLeft, frontRight, backLeft, backRight
        };

        int[] targets = {
                flTicks, frTicks, blTicks, brTicks
        };

        for (int i = 0; i < motors.length; i++) {
            motors[i].setTargetPosition(
                    motors[i].getCurrentPosition() + targets[i]);
            motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        for (DcMotor motor : motors) {
            motor.setPower(power);
        }

        double timeoutSeconds = 2.0 + distanceInches / 18.0;

        ElapsedTime movementTimer = new ElapsedTime();
        movementTimer.reset();

        while (opModeIsActive()
                && canContinue()
                && anyMotorBusy()
                && movementTimer.seconds() < timeoutSeconds) {

            telemetry.addData("Movement", "Running");
            telemetry.addData("Movement time", "%.1f",
                    movementTimer.seconds());
            telemetry.addData("Match time", "%.1f",
                    matchTimer.seconds());

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

        boolean timedOut =
                movementTimer.seconds() >= timeoutSeconds
                        && anyMotorBusy();

        stopDrive();

        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        return !timedOut && canContinue();
    }

    private boolean anyMotorBusy() {

        return frontLeft.isBusy()
                || frontRight.isBusy()
                || backLeft.isBusy()
                || backRight.isBusy();
    }

    private boolean waitSafely(long milliseconds) {

        ElapsedTime waitTimer = new ElapsedTime();
        waitTimer.reset();

        while (opModeIsActive()
                && canContinue()
                && waitTimer.milliseconds() < milliseconds) {

            telemetry.addData("Waiting", "%.1f seconds",
                    waitTimer.seconds());
            telemetry.addData("Match time", "%.1f seconds",
                    matchTimer.seconds());
            telemetry.update();

            idle();
        }

        return canContinue();
    }

    private boolean canContinue() {
        return opModeIsActive()
                && matchTimer.seconds() < MATCH_LIMIT_SECONDS;
    }

    private void stopDrive() {

        if (frontLeft != null) frontLeft.setPower(0);
        if (frontRight != null) frontRight.setPower(0);
        if (backLeft != null) backLeft.setPower(0);
        if (backRight != null) backRight.setPower(0);
    }
}

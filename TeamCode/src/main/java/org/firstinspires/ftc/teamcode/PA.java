
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Pollen Auto", group = "Autonomous")
public class PA extends LinearOpMode {

    // ================= HARDWARE =================

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // ================= CALIBRATION =================

    private static final double TICKS_PER_INCH = 41.81;
    private static final double TURN_DIAMETER_INCHES = 28.0;

    // ================= POWER =================

    private static final double DRIVE_POWER = 0.25;
    private static final double TURN_POWER = 0.22;

    // ================= SAFETY =================

    private static final double MOVE_TIMEOUT_BASE = 2.0;
    private static final double MAX_MOVE_TIMEOUT = 8.0;
    private static final double MATCH_LIMIT = 29.0;

    private final ElapsedTime matchTimer = new ElapsedTime();
    private final ElapsedTime moveTimer = new ElapsedTime();

    // =================================================
    // MAIN
    // =================================================

    @Override
    public void runOpMode() {

        if (!initializeHardware()) {
            telemetry.addLine("HARDWARE ERROR");
            telemetry.update();
            return;
        }

        telemetry.addLine("POLLEN AUTONOMOUS");
        telemetry.addLine("Movement-only configuration");
        telemetry.addData("Ticks/Inch", TICKS_PER_INCH);
        telemetry.addData("Turn Diameter", TURN_DIAMETER_INCHES);
        telemetry.addLine("Shooter and intake disabled");
        telemetry.addLine("Route is provisional - verify before field use");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            stopDrive();
            return;
        }

        matchTimer.reset();

        runPollenRoute();

        stopDrive();

        telemetry.addLine("POLLEN AUTO FINISHED");
        telemetry.update();
    }

    // =================================================
    // HARDWARE INITIALIZATION
    // =================================================

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

            telemetry.addData("Hardware Error", e.getMessage());
            telemetry.update();

            return false;
        }

        // Previously tested motor directions

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        DcMotor[] motors = getMotors();

        for (DcMotor motor : motors) {

            motor.setPower(0);

            motor.setZeroPowerBehavior(
                    DcMotor.ZeroPowerBehavior.BRAKE);

            motor.setMode(
                    DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            motor.setMode(
                    DcMotor.RunMode.RUN_USING_ENCODER);
        }

        return true;
    }

    private DcMotor[] getMotors() {

        return new DcMotor[]{
                frontLeft,
                frontRight,
                backLeft,
                backRight
        };
    }

    // =================================================
    // MECANUM DRIVE
    // =================================================

    /*
     * Positive forward = forward.
     * Positive strafeRight = right.
     *
     * These are the original equations tested successfully
     * on this robot. Do not reverse their signs.
     */

    private boolean drive(
            double forward,
            double strafeRight) {

        int flTicks = (int) Math.round(
                (forward + strafeRight) * TICKS_PER_INCH);

        int frTicks = (int) Math.round(
                (forward - strafeRight) * TICKS_PER_INCH);

        int blTicks = (int) Math.round(
                (forward - strafeRight) * TICKS_PER_INCH);

        int brTicks = (int) Math.round(
                (forward + strafeRight) * TICKS_PER_INCH);

        return runToPosition(
                flTicks,
                frTicks,
                blTicks,
                brTicks,
                DRIVE_POWER,
                "DRIVE"
        );
    }

    // =================================================
    // TURNING
    // =================================================

    /*
     * Positive degrees = clockwise.
     * Negative degrees = counterclockwise.
     */

    private boolean turn(double degrees) {

        double wheelTravel =
                Math.PI * TURN_DIAMETER_INCHES
                        * degrees / 360.0;

        int ticks = (int) Math.round(
                wheelTravel * TICKS_PER_INCH);

        return runToPosition(
                ticks,
                -ticks,
                ticks,
                -ticks,
                TURN_POWER,
                "TURN " + degrees
        );
    }

    // =================================================
    // ENCODER MOVEMENT ENGINE
    // =================================================

    private boolean runToPosition(
            int flDelta,
            int frDelta,
            int blDelta,
            int brDelta,
            double power,
            String action) {

        if (!opModeIsActive() || timeLimitReached()) {
            stopDrive();
            return false;
        }

        DcMotor[] motors = getMotors();

        int[] deltas = {
                flDelta,
                frDelta,
                blDelta,
                brDelta
        };

        int maxTicks = 0;

        // Set target positions

        for (int i = 0; i < motors.length; i++) {

            int target =
                    motors[i].getCurrentPosition() + deltas[i];

            motors[i].setTargetPosition(target);

            maxTicks = Math.max(
                    maxTicks,
                    Math.abs(deltas[i]));
        }

        // Activate RUN_TO_POSITION

        for (DcMotor motor : motors) {

            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motor.setPower(Math.abs(power));
        }

        // Calculate timeout

        double estimatedInches =
                maxTicks / TICKS_PER_INCH;

        double timeoutSeconds = Math.min(
                MAX_MOVE_TIMEOUT,
                MOVE_TIMEOUT_BASE + estimatedInches / 5.0
        );

        moveTimer.reset();

        // Movement monitoring loop

        while (opModeIsActive()) {

            boolean anyMotorBusy = false;

            for (DcMotor motor : motors) {

                if (motor.isBusy()) {
                    anyMotorBusy = true;
                    break;
                }
            }

            // Telemetry

            telemetry.addData("Action", action);

            telemetry.addData(
                    "Match Time",
                    "%.1f / %.1f",
                    matchTimer.seconds(),
                    MATCH_LIMIT
            );

            telemetry.addData(
                    "Movement Time",
                    "%.1f / %.1f",
                    moveTimer.seconds(),
                    timeoutSeconds
            );

            telemetry.addData(
                    "Front Left",
                    "%d / %d",
                    frontLeft.getCurrentPosition(),
                    frontLeft.getTargetPosition()
            );

            telemetry.addData(
                    "Front Right",
                    "%d / %d",
                    frontRight.getCurrentPosition(),
                    frontRight.getTargetPosition()
            );

            telemetry.addData(
                    "Back Left",
                    "%d / %d",
                    backLeft.getCurrentPosition(),
                    backLeft.getTargetPosition()
            );

            telemetry.addData(
                    "Back Right",
                    "%d / %d",
                    backRight.getCurrentPosition(),
                    backRight.getTargetPosition()
            );

            telemetry.update();

            // Safety timeout

            if (timeLimitReached()
                    || moveTimer.seconds() > timeoutSeconds) {

                stopDrive();

                telemetry.addLine(
                        "MOVEMENT STOPPED: TIMEOUT");

                telemetry.update();

                return false;
            }

            // All motors reached their targets

            if (!anyMotorBusy) {

                stopDrive();

                return true;
            }

            idle();
        }

        stopDrive();

        return false;
    }

    // =================================================
    // TIME LIMIT
    // =================================================

    private boolean timeLimitReached() {

        return matchTimer.seconds() >= MATCH_LIMIT;
    }

    // =================================================
    // STOP ALL MOTORS
    // =================================================

    private void stopDrive() {

        if (frontLeft == null
                || frontRight == null
                || backLeft == null
                || backRight == null) {

            return;
        }

        for (DcMotor motor : getMotors()) {

            motor.setPower(0);

            motor.setMode(
                    DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    // =================================================
    // POLLEN AUTONOMOUS ROUTE
    // =================================================

    private void runPollenRoute() {

        telemetry.addLine("STARTING POLLEN ROUTE");
        telemetry.update();

        // 1. Move forward 20 inches

        if (!drive(20, 0)) return;

        // 2. Turn clockwise 35 degrees

        if (!turn(35)) return;

        // 3. Move forward 18 inches

        if (!drive(18, 0)) return;

        // 4. Turn counterclockwise 70 degrees

        if (!turn(-70)) return;

        // 5. Move forward 30 inches

        if (!drive(30, 0)) return;

        /*
         * Shooter, intake, pollen release, and collection
         * are intentionally not implemented yet.
         *
         * The recovered route is provisional.
         * Verify starting position, heading, and distances
         * before running on the competition field.
         */
    }
}

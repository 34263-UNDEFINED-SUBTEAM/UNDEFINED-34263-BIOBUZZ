
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name = "Backup Strategy Park Right", group = "Competition")
public class Backup_Strategy_Park_Right extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private GoBildaPinpointDriver pinpoint;

    private static final String PINPOINT_NAME = "pinpoint";

    // Pinpoint offsets: these are assumptions and must match your mounting.
    private static final double X_OFFSET_MM = 0.0;
    private static final double Y_OFFSET_MM = 0.0;

    private static final double MAX_DRIVE_POWER = 0.40;
    private static final double MIN_DRIVE_POWER = 0.12;
    private static final double DRIVE_KP = 0.10;
    private static final double DRIVE_HEADING_KP = 0.012;

    private static final double TURN_KP = 0.012;
    private static final double MAX_TURN_POWER = 0.30;
    private static final double MIN_TURN_POWER = 0.08;

    private static final double POSITION_TOLERANCE = 0.35;
    private static final double HEADING_TOLERANCE = 1.0;

    private static final double DRIVE_TIMEOUT = 10.0;
    private static final double TURN_TIMEOUT = 7.0;

    @Override
    public void runOpMode() {

        telemetry.addLine("BACKUP STRATEGY PARK RIGHT");
        telemetry.addLine("Initializing...");
        telemetry.update();

        try {
            frontLeft = hardwareMap.get(DcMotor.class, "front_left_drive");
            frontRight = hardwareMap.get(DcMotor.class, "front_right_drive");
            backLeft = hardwareMap.get(DcMotor.class, "back_left_drive");
            backRight = hardwareMap.get(DcMotor.class, "back_right_drive");

            pinpoint = hardwareMap.get(
                    GoBildaPinpointDriver.class, PINPOINT_NAME
            );

        } catch (Exception e) {
            telemetry.addData("HARDWARE ERROR", e.getMessage());
            telemetry.update();
            return;
        }

        // Drive motor directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        DcMotor[] motors = {
                frontLeft, frontRight, backLeft, backRight
        };

        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(0);
        }

        // Pinpoint setup
        pinpoint.setOffsets(X_OFFSET_MM, Y_OFFSET_MM, DistanceUnit.MM);

        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );

        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        telemetry.addLine("Calibrating Pinpoint.");
        telemetry.addLine("Keep robot stationary.");
        telemetry.update();

        pinpoint.resetPosAndIMU();
        sleep(500);
        pinpoint.update();

        telemetry.addLine("READY");
        telemetry.addLine("Forward 17 inches");
        telemetry.addLine("Turn left 90 degrees");
        telemetry.addLine("Forward 5 inches");
        telemetry.addData(
                "Heading",
                "%.2f",
                getHeading()
        );
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            stopDrive();
            return;
        }

        // STEP 1
        telemetry.addLine("STEP 1: FORWARD 17 INCHES");
        telemetry.update();

        if (!driveForward(17)) {
            finish("ERROR: FIRST DRIVE FAILED");
            return;
        }

        // STEP 2
        telemetry.addLine("STEP 2: TURN LEFT 90 DEGREES");
        telemetry.update();

        if (!turnLeft(90)) {
            finish("ERROR: TURN FAILED");
            return;
        }

        // STEP 3
        telemetry.addLine("STEP 3: FORWARD 5 INCHES");
        telemetry.update();

        if (!driveForward(5)) {
            finish("ERROR: SECOND DRIVE FAILED");
            return;
        }

        finish("AUTONOMOUS COMPLETE");
    }

    // --------------------------------------------------
    // FORWARD DRIVE USING PINPOINT POSITION
    // --------------------------------------------------

    private boolean driveForward(double inches) {

        pinpoint.update();

        Pose2D start = pinpoint.getPosition();

        double startX = start.getX(DistanceUnit.INCH);
        double startY = start.getY(DistanceUnit.INCH);
        double startHeading = getHeading();

        double headingRadians = Math.toRadians(startHeading);

        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && timer.seconds() < DRIVE_TIMEOUT) {

            pinpoint.update();

            Pose2D current = pinpoint.getPosition();

            double x = current.getX(DistanceUnit.INCH);
            double y = current.getY(DistanceUnit.INCH);

            double traveled =
                    (x - startX) * Math.cos(headingRadians)
                            + (y - startY) * Math.sin(headingRadians);

            double remaining = inches - traveled;

            if (remaining <= POSITION_TOLERANCE) {
                stopDrive();
                return true;
            }

            double power = Math.max(
                    MIN_DRIVE_POWER,
                    Math.min(MAX_DRIVE_POWER, remaining * DRIVE_KP)
            );

            double headingError =
                    normalizeAngle(startHeading - getHeading());

            double correction = clip(
                    headingError * DRIVE_HEADING_KP,
                    -0.12,
                    0.12
            );

            // All four wheels receive forward power.
            setDrivePower(
                    power - correction,
                    power + correction,
                    power - correction,
                    power + correction
            );

            telemetry.addData("Action", "FORWARD");
            telemetry.addData("Target", "%.1f in", inches);
            telemetry.addData("Traveled", "%.2f in", traveled);
            telemetry.addData("Remaining", "%.2f in", remaining);
            telemetry.addData("Heading", "%.2f", getHeading());
            telemetry.update();

            idle();
        }

        stopDrive();
        return false;
    }

    // --------------------------------------------------
    // PINPOINT TURN
    // --------------------------------------------------

    private boolean turnLeft(double degrees) {

        double startingHeading = getHeading();

        /*
         * First send a short left-turn pulse.
         * This checks whether Pinpoint detects rotation.
         */
        setTurnPower(0.22);
        sleep(350);
        stopDrive();

        if (!opModeIsActive()) {
            return false;
        }

        pinpoint.update();

        double pulseHeading = getHeading();
        double headingChange =
                normalizeAngle(pulseHeading - startingHeading);

        // If Pinpoint detects practically no rotation, abort.
        if (Math.abs(headingChange) < 0.5) {
            telemetry.addLine("TURN ABORTED");
            telemetry.addLine("Pinpoint detected almost no rotation.");
            telemetry.addLine("Check motor directions and wiring.");
            telemetry.update();
            return false;
        }

        /*
         * The initial motor pattern establishes the physical
         * turn direction. Pinpoint's heading sign may differ
         * depending on its coordinate convention.
         */
        double headingSign = Math.signum(headingChange);

        double targetHeading = normalizeAngle(
                pulseHeading + headingSign * (degrees - 0.5)
        );

        ElapsedTime timer = new ElapsedTime();
        ElapsedTime settleTimer = new ElapsedTime();

        boolean settling = false;

        while (opModeIsActive() && timer.seconds() < TURN_TIMEOUT) {

            pinpoint.update();

            double currentHeading = getHeading();

            double error = normalizeAngle(
                    targetHeading - currentHeading
            );

            if (Math.abs(error) <= HEADING_TOLERANCE) {

                stopDrive();

                if (!settling) {
                    settleTimer.reset();
                    settling = true;
                }

                if (settleTimer.seconds() >= 0.25) {
                    stopDrive();
                    return true;
                }

            } else {

                settling = false;

                double magnitude = Math.min(
                        MAX_TURN_POWER,
                        Math.abs(error) * TURN_KP
                );

                magnitude = Math.max(
                        MIN_TURN_POWER,
                        magnitude
                );

                /*
                 * Keep the initial physical turn direction
                 * while error remains in that direction.
                 * Reverse only if the target is overshot.
                 */
                double output =
                        Math.signum(error) * headingSign * magnitude;

                setTurnPower(output);
            }

            telemetry.addData("Action", "TURN");
            telemetry.addData("Start", "%.2f", startingHeading);
            telemetry.addData("Target", "%.2f", targetHeading);
            telemetry.addData("Current", "%.2f", currentHeading);
            telemetry.addData("Error", "%.2f", error);
            telemetry.addData("Turn direction", "%.0f", headingSign);
            telemetry.update();

            idle();
        }

        stopDrive();
        return false;
    }

    // --------------------------------------------------
    // TURN MOTOR PATTERN
    // --------------------------------------------------

    private void setTurnPower(double power) {

        /*
         * Opposite power on each side:
         * left side backward, right side forward = left turn.
         */
        setDrivePower(
                -power,
                power,
                -power,
                power
        );
    }

    // --------------------------------------------------
    // HELPERS
    // --------------------------------------------------

    private double getHeading() {
        pinpoint.update();
        return pinpoint.getPosition().getHeading(AngleUnit.DEGREES);
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    private double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private void setDrivePower(
            double fl,
            double fr,
            double bl,
            double br
    ) {
        frontLeft.setPower(clip(fl, -1, 1));
        frontRight.setPower(clip(fr, -1, 1));
        backLeft.setPower(clip(bl, -1, 1));
        backRight.setPower(clip(br, -1, 1));
    }

    private void stopDrive() {
        if (frontLeft != null) frontLeft.setPower(0);
        if (frontRight != null) frontRight.setPower(0);
        if (backLeft != null) backLeft.setPower(0);
        if (backRight != null) backRight.setPower(0);
    }

    private void finish(String message) {
        stopDrive();
        telemetry.addLine(message);
        telemetry.update();
    }
}

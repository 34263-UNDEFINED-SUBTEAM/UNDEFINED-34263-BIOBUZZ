
        package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "TEST SOL AUTO", group = "Autonomous")
public class TEST_SOL_AUTO extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private SparkFunOTOS otos;

    private static final double SIDE_LENGTH = 24.0;
    private static final double MAX_POWER = 0.45;
    private static final double MIN_POWER = 0.12;
    private static final double POSITION_TOLERANCE = 1.0;
    private static final double HEADING_TOLERANCE = 3.0;
    private static final double HEADING_KP = 0.012;

    @Override
    public void runOpMode() {

        frontLeft = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeft = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRight = hardwareMap.get(DcMotor.class, "back_right_drive");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        otos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");

        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);
        otos.setOffset(new SparkFunOTOS.Pose2D(0, 0, 0));
        otos.setLinearScalar(1.0);
        otos.setAngularScalar(1.0);

        telemetry.addLine("Keep robot still during calibration");
        telemetry.update();

        otos.calibrateImu();
        otos.resetTracking();
        otos.setPosition(new SparkFunOTOS.Pose2D(0, 0, 0));

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            moveTo(SIDE_LENGTH, 0);
            moveTo(SIDE_LENGTH, SIDE_LENGTH);
            moveTo(0, SIDE_LENGTH);
            moveTo(0, 0);
        }

        stopMotors();
    }

    private void moveTo(double targetX, double targetY) {

        double startTime = getRuntime();

        while (opModeIsActive() && getRuntime() - startTime < 8.0) {

            SparkFunOTOS.Pose2D pose = otos.getPosition();

            double errorX = targetX - pose.x;
            double errorY = targetY - pose.y;
            double distance = Math.hypot(errorX, errorY);
            double headingError = wrapAngle(-pose.h);

            if (distance < POSITION_TOLERANCE
                    && Math.abs(headingError) < HEADING_TOLERANCE) {
                stopMotors();
                return;
            }

            double heading = Math.toRadians(pose.h);

            double forward =
                    errorX * Math.cos(heading)
                            + errorY * Math.sin(heading);

            double strafe =
                    -errorX * Math.sin(heading)
                            + errorY * Math.cos(heading);

            if (distance > 0.001) {
                forward /= distance;
                strafe /= distance;
            }

            double power = Math.min(
                    MAX_POWER,
                    Math.max(MIN_POWER, distance * 0.04)
            );

            forward *= power;
            strafe *= power;

            double turn = Math.max(
                    -0.20,
                    Math.min(0.20, headingError * HEADING_KP)
            );

            double fl = forward + strafe + turn;
            double fr = forward - strafe - turn;
            double bl = forward - strafe + turn;
            double br = forward + strafe - turn;

            double max = Math.max(
                    1.0,
                    Math.max(
                            Math.max(Math.abs(fl), Math.abs(fr)),
                            Math.max(Math.abs(bl), Math.abs(br))
                    )
            );

            frontLeft.setPower(fl / max);
            frontRight.setPower(fr / max);
            backLeft.setPower(bl / max);
            backRight.setPower(br / max);

            telemetry.addData("X", "%.2f", pose.x);
            telemetry.addData("Y", "%.2f", pose.y);
            telemetry.addData("Heading", "%.1f", pose.h);
            telemetry.addData("Distance remaining", "%.2f", distance);
            telemetry.addData("Heading error", "%.1f", headingError);
            telemetry.update();
        }

        stopMotors();

        telemetry.addLine("Movement timed out");
        telemetry.update();
    }

    private double wrapAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    private void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
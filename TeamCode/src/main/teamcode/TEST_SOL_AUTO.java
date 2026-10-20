package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
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
    private Limelight3A limelight;

    private static final double SIDE = 24.0;
    private static final double TOLERANCE = 1.0;
    private static final double MAX_POWER = 0.45;
    private static final double MIN_POWER = 0.12;

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
        otos.calibrateImu();
        otos.resetTracking();
        otos.setPosition(new SparkFunOTOS.Pose2D(0, 0, 0));

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addLine("Ready to run");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            moveTo(SIDE, 0);
            moveTo(SIDE, SIDE);
            moveTo(0, SIDE);
            moveTo(0, 0);
        }

        stopMotors();
        limelight.stop();
    }

    private void moveTo(double targetX, double targetY) {

        double startTime = getRuntime();

        while (opModeIsActive() && getRuntime() - startTime < 8.0) {

            SparkFunOTOS.Pose2D pose = otos.getPosition();

            double errorX = targetX - pose.x;
            double errorY = targetY - pose.y;
            double distance = Math.hypot(errorX, errorY);

            if (distance < TOLERANCE) {
                stopMotors();
                return;
            }

            double angle = Math.toRadians(pose.h);

            double forward =
                    errorX * Math.cos(angle)
                            + errorY * Math.sin(angle);

            double strafe =
                    -errorX * Math.sin(angle)
                            + errorY * Math.cos(angle);

            forward /= distance;
            strafe /= distance;

            double power = Math.min(
                    MAX_POWER,
                    Math.max(MIN_POWER, distance * 0.04)
            );

            forward *= power;
            strafe *= power;

            double fl = forward + strafe;
            double fr = forward - strafe;
            double bl = forward - strafe;
            double br = forward + strafe;

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

            LLResult result = limelight.getLatestResult();

            telemetry.addData("OTOS X", "%.2f", pose.x);
            telemetry.addData("OTOS Y", "%.2f", pose.y);
            telemetry.addData("Heading", "%.1f", pose.h);
            telemetry.addData("Distance remaining", "%.2f", distance);

            if (result != null && result.isValid()) {
                telemetry.addData("Limelight", "Target found");
                telemetry.addData("Target X", "%.2f", result.getTx());
            } else {
                telemetry.addData("Limelight", "No target");
            }

            telemetry.update();
        }

        stopMotors();
    }

    private void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}

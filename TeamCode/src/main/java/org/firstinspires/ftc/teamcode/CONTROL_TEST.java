package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.List;

@Autonomous(name = "CONTROL TEST", group = "Autonomous")
public class CONTROL_TEST extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private Limelight3A limelight;

    private static final double DRIVE_POWER = 0.4;

    @Override
    public void runOpMode() throws InterruptedException {

        frontLeft = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeft = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRight = hardwareMap.get(DcMotor.class, "back_right_drive");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);
        limelight.start();

        stopRobot();

        telemetry.addLine("Ready");
        telemetry.addLine("Looking for AprilTags 42 and 43");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();

            boolean targetFound = false;

            if (result != null && result.isValid()) {

                List<LLResultTypes.FiducialResult> tags =
                        result.getFiducialResults();

                for (LLResultTypes.FiducialResult tag : tags) {

                    int id = tag.getFiducialId();

                    telemetry.addData("Tag ID", id);

                    if (id == 42 || id == 43) {
                        targetFound = true;
                    }
                }
            }

            if (targetFound) {

                frontLeft.setPower(DRIVE_POWER);
                frontRight.setPower(DRIVE_POWER);
                backLeft.setPower(DRIVE_POWER);
                backRight.setPower(DRIVE_POWER);

                telemetry.addLine("42/43 FOUND -> FORWARD");

            } else {

                stopRobot();
                telemetry.addLine("Waiting for AprilTag 42 or 43");
            }

            telemetry.update();
        }

        stopRobot();
    }

    private void stopRobot() {

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
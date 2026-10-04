package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.List;

@TeleOp(name = "Limelight 3.1 BackUp", group = "Autonomous")
public class Limelight31BackUp extends LinearOpMode {

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private Limelight3A limelight;

    private static final double BACKWARD_SPEED = -0.2;

    @Override
    public void runOpMode() {
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            boolean targetSeen = false;

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> aprilTags = result.getFiducialResults();

                for (LLResultTypes.FiducialResult tag : aprilTags) {
                    if (tag.getFiducialId() == 30 || tag.getFiducialId() == 31) {
                        targetSeen = true;
                        break;
                    }
                }
            }

            if (targetSeen) {
                leftDrive.setPower(BACKWARD_SPEED);
                rightDrive.setPower(BACKWARD_SPEED);
            } else {
                leftDrive.setPower(0);
                rightDrive.setPower(0);
            }
        }

        limelight.stop();
    }
}
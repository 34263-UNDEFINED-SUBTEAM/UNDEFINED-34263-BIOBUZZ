package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TfodCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.lang.Math;
import java.util.Set;
import java.util.List;
import java.lang.annotation.Target;

@Autonomous(name="FTC IMU code")
public class FTC_IMU_code extends LinearOpMode
{
    // Set Timer 
    private ElapsedTime  runtime = new ElapsedTime();
    
    
    // declare motors
    private DcMotor frontright;
    private DcMotor frontleft;
    private DcMotor backright;
    private DcMotor backleft;

    //declare IMU and Huskylens
   
    private BNO055IMU imu;

    private HuskyLens huskyLens;
    
    // encoder clicks are originally 1680
    // multiply that by drive train gear ratio (two thirds)   
    private static final int ENCODER_CLICKS = 28; //HD hex motor
    private static final double WHEEL_DIAM = 9.6; //gobuilda in CM
    private static final double WHEEL_CIRC = WHEEL_DIAM * Math.PI;
    private static final double DRIVE_GEAR_RATIO = 2/3;
    private static final double CLICKS_PER_CM = ENCODER_CLICKS / WHEEL_CIRC;

    // Declare IMU 
    BNO055IMU.Parameters IMU_Parameters;
    ElapsedTime ElapsedTime2;
    double Left_Power;
    double Right_Power;
    double Original_Power;
    float Yaw_Angle = 0;   
    
    //declare tracking position
    double pos;
    
    
    @Override
    public void runOpMode()
    {
        // Telemetry to show OpMode is initialized
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        






        // Initialize the hardware variables. 
        backleft  = hardwareMap.get(DcMotor.class, "back_left_drive");
        backright = hardwareMap.get(DcMotor.class, "back_right_drive");
        frontleft  = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontright = hardwareMap.get(DcMotor.class, "front_right_drive");
   
         imu = hardwareMap.get(BNO055IMU.class, "imu");
        
        // Initialize motor direction
        backright.setDirection(DcMotor.Direction.FORWARD);
        backleft.setDirection(DcMotor.Direction.REVERSE);
        frontleft.setDirection(DcMotor.Direction.REVERSE);
        frontright.setDirection(DcMotor.Direction.FORWARD);
      
        
    
        // Reset motor encoders
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // Initialize motor encoder modes
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        
        
        
        // Initialize IMU 
        IMU_Parameters = new BNO055IMU.Parameters();
        IMU_Parameters.mode = BNO055IMU.SensorMode.IMU;
        imu.initialize(IMU_Parameters);   
        
        telemetry.addData("Status", "IMU initialized, calibration started.");
        telemetry.update();
        
         // Calibrate IMU       
        while (!IMU_Calibrated()) {
          telemetry.addData("If calibration ", "doesn't complete after 3 seconds, move through 90 degree pitch, roll and yaw motions until calibration complete ");
          telemetry.update();
          // Wait one second before checking calibration status again.
          sleep(1000);
        }
           
        telemetry.addData("Status", "Calibration Complete");
        telemetry.update();
        
        
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        

        telemetry.addData("status","init" );
        telemetry.update();


        // OpMode needs waitForStart(); and while (opModeIsActive()) {...}
        waitForStart();

        telemetry.addData("status","op mode started" );
        telemetry.update();
        
        
//Drive forward to detecting position

driveBot(90,90,0.4,5.0);
            telemetry.addData("status","Detecting" );
            telemetry.addData("status", left.getMode() );
            telemetry.addData("status","left motor,  %7d", left.getCurrentPosition() );
            telemetry.addData("status","right motor,  %7d", right.getCurrentPosition() );
            telemetry.update();
            right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(1000);
//First stage detection here


        
        
        //Automonous Drive Code Starts Here
       
        if (Pos == 1) {
            //position 1 autonomous code
            
            
        }
       //end
        
        if (pos == 2) {
            //position 2 autonomous code


        }
       //end
        
        if (pos == 3) {
            //position 3 autonomous code


        }
       //end

    }
    // END runOpMode()
    
    
    
    
    /**********************************************
     * BEGIN METHODS driveDistance, driveBot
     *********************************************/
    
    /**
    * Drive Distance Method
    * Calculate distance from CM to encoder counts
    */  
    public static double driveDistance(double distance)
    {
        double drive  = (ENCODER_CLICKS/ WHEEL_CIRC);
        int outputClicks= (int)Math.floor(drive * distance);
        return outputClicks;
    }
    // END driveDistance() method


    /**
    * Drive Method
    * Left & Right travel distance in CM +/-, power to both wheels, timeout in seconds
    */
    public void driveBot(double distanceInCMleft, double distanceInCMright, double power, double timeoutS) 
    {
        telemetry.addData("status","encoder reset");
        telemetry.update();
        
        int rightTarget;
        int leftTarget;

        if(opModeIsActive()) 
        {
            telemetry.addData("status","getEncoderClicks");
            telemetry.update();
            
            rightTarget = (int) driveDistance(distanceInCMright);
            leftTarget = (int) driveDistance(distanceInCMleft);

            right.setTargetPosition(rightTarget);
            left.setTargetPosition(leftTarget);

            right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
            runtime.reset();

            right.setPower(power);
            left.setPower(power);
        
            while (opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (left.isBusy() && right.isBusy()))
                {
                    telemetry.addData("Path1", "leftTarget, rightTarget" );
                    telemetry.update();
                }
            left.setPower(0);
            right.setPower(0);
            left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }   
    }
    // END driveBot method
    
    
    /**
    * Turn left with IMU
    * Left travel distance in degrees  +/-, power to both wheels, timeout in seconds
    */
    public void turnLeft()
    {
        //MOVE RIGHT
        left.setPower(-0.2);
        right.setPower(0.2);
        
        // Continue until robot yaws right by 90 degrees or stop is pressed on Driver Station.
        sleep(1000);
        //turns to the right; 90, 180, negative, -90, 0
        while ( !(Yaw_Angle <= -85 || isStopRequested()) ) {
            // Update Yaw-Angle variable with current yaw.
            Yaw_Angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            // Report yaw orientation to Driver Station.
            telemetry.addData("Yaw value", Yaw_Angle);
            telemetry.update();
        }
        // We're done. Turn off motors
        left.setPower(0);
        right.setPower(0);
        // Pause so final telemetry is displayed.
        sleep(1000);
    }
    // END turnLeft
    

    /**
    * Turn right with IMU
    * Left & Right travel distance in degrees +/-, power to both wheels, timeout in seconds
    */
    public void turnRight()
    {
        //MOVE RIGHT
        left.setPower(0.2);
        right.setPower(-0.2);
        // Continue until robot yaws right by 90 degrees or stop is pressed on Driver Station.
        sleep(1000);
        //turns to the right; 90, 180, negative, -90, 0
        while ( !(Yaw_Angle >= 85 || isStopRequested()) ) {
            // Update Yaw-Angle variable with current yaw.
            Yaw_Angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            // Report yaw orientation to Driver Station.
            telemetry.addData("Yaw value", Yaw_Angle);
            telemetry.update();
        }
        // We're done. Turn off motors
        left.setPower(0);
        right.setPower(0);
        // Pause so final telemetry is displayed.
        sleep(1000);
    }
    // END turnRight method
    
    
    /**
    * Turn  with IMU
    * true for right, degrees are for the direction you're going in
    * Left & Right travel distance in degrees +/-, power to both wheels, timeout in seconds
    */
    public void turn(boolean dir, int degrees)
    {
        if(!dir)
        {
            //MOVE LEFT
            left.setPower(-0.2);
            right.setPower(0.2);
            degrees = (int)(degrees * 0.94444444444);

            //turns to the right; 90, 180, negative, -90, 0
            sleep(1000);
            while ( !(Yaw_Angle <= (-1*degrees) || isStopRequested()) ) {
                // Update Yaw-Angle variable with current yaw.
                Yaw_Angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
                // Report yaw orientation to Driver Station.
                telemetry.addData("Yaw value", Yaw_Angle);
                telemetry.update();
            }
        }
        else
        {
            //MOVE RIGHT
            left.setPower(0.2);
            right.setPower(-0.2);
            degrees = (int)(degrees * 0.94444444444);

            sleep(1000);
            while ( !(Yaw_Angle >= degrees || isStopRequested()) ) {
                // Update Yaw-Angle variable with current yaw.
                Yaw_Angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
                // Report yaw orientation to Driver Station.
                telemetry.addData("Yaw value", Yaw_Angle);
                telemetry.update();
            }
        }
        // We're done. Turn off motors
        left.setPower(0);
        right.setPower(0);
        // Pause so final telemetry is displayed.
        sleep(1000);
    }
    // END turn method
    
    
    /**
    * IMU Calibration Method
    * Checks IMU calibration and returns telementry
    * If IMU is NOT calibrated, run the calibration opMode 
    */ 
        private boolean IMU_Calibrated() {
        telemetry.addData("IMU Calibration Status", imu.getCalibrationStatus());
        telemetry.addData("Gyro Calibrated", imu.isGyroCalibrated() ? "True" : "False");
        telemetry.addData("System Status", imu.getSystemStatus().toString());
        return imu.isGyroCalibrated();
    }
}
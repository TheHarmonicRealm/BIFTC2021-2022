// base: /FtcRobotController\src\main\java\org\firstinspires\ftc\robotcontroller\external\samples\BasicOpMode_Iterative.java

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//import org.firstinspires.ftc.teamcode.Constants.BasicBot;

/**
 * This file is an iterative (Non-Linear) "OpMode" for TeleOp driving.
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot with four motors.
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 */

@TeleOp(name="BasicBot: Mecanum Drive", group="Mecanum Drive")
public class BasicBot_MecanumDrive extends OpMode
{
    // Declare OpMode members.
    public DcMotor  leftFrontDrive;
    public DcMotor  leftBackDrive;
    public DcMotor  rightFrontDrive;
    public DcMotor  rightBackDrive;

    public static final double kLeftDeadZoneY = .1;
    public static final double kRightDeadZoneY = .1;

    private ElapsedTime runtime = new ElapsedTime();

    public class MotorSpeeds{
        double leftFront;
        double rightFront;
        double leftBack;
        double rightBack;

        public MotorSpeeds(double lF, double rF, double lB, double rB) {
            leftFront = lF;
            rightFront = rF;
            leftBack = lB;
            rightBack = rB;
        }
    }

    public double getInputWeight(double primary, double secondary, double tertiary) {

        primary = Math.abs(primary);
        secondary = Math.abs(secondary);
        tertiary = Math.abs(tertiary);
        return primary/(primary + secondary + tertiary);
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        //BasicBot_Hardware.java includes all the hardware inits. This runs them.
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD); // Set to FORWARD if using AndyMark motors
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD); // Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftFrontPower;
        double rightFrontPower;
        double leftBackPower;
        double rightBackPower;

        //Get the values of the joysticks in this loop. A very very large number of loops happen every second
        double leftY = gamepad1.left_stick_y;
        double rightX  =  gamepad1.right_stick_X;
        double leftX = gamepad1.left_stick_x;

        //If sticks somehow are over 1/under -1 clip to those (max motor levels)
        leftY = Range.clip(leftY, -1.0, 1.0);
        leftX = Range.clip(leftX, -1.0, 1.0);
        rightX = Range.clip(rightX, -1.0, 1.0);

        //If the sticks are within a "deadzone" defined in Constants.java (.1 range initially) set them to 0
        if(Math.abs(leftY) <= kLeftDeadZoneY){
            leftY = 0;
        }
        if(Math.abs(rightX) <= kRightDeadZoneY){
            rightX = 0;
        }

        MotorSpeeds forwardSpeeds = new MotorSpeeds(leftY, leftY, leftY, leftY);

        MotorSpeeds sidewaysSpeeds = new MotorSpeeds(-leftX, leftX, leftX, -leftX);

        MotorSpeeds rotatingSpeeds = new MotorSpeeds(rightX, -rightX, rightX, -rightX);

        double leftFrontCombined = forwardSpeeds.leftFront*getInputWeight(leftY, leftX, rightX)
            +sidewaysSpeeds.leftFront*getInputWeight(leftX, leftY, rightX)
            +rotatingSpeeds.leftFront*getInputWeight(rightX, leftY, leftX);

        double rightBackCombined = forwardSpeeds.rightBack*getInputWeight(leftY, leftX, rightX)
            +sidewaysSpeeds.rightBack*getInputWeight(leftX, leftY, rightX)
            +rotatingSpeeds.rightBack*getInputWeight(rightX, leftY, leftX);

        double leftBackCombined = forwardSpeeds.leftBack*getInputWeight(leftY, leftX, rightX)
            +sidewaysSpeeds.leftBack*getInputWeight(leftX, leftY, rightX)
            +rotatingSpeeds.leftBack*getInputWeight(rightX, leftY, leftX);

        double rightFrontCombined = forwardSpeeds.rightFront*getInputWeight(leftY, leftX, rightX)
            +sidewaysSpeeds.rightFront*getInputWeight(leftX, leftY, rightX)
            +rotatingSpeeds.rightFront*getInputWeight(rightX, leftY, leftX);

        MotorSpeeds combinedSpeeds = new MotorSpeeds(leftFrontCombined, rightFrontCombined, leftBackCombined, rightBackCombined);

        //Send the values to the motors
        leftFrontDrive.setPower(combinedSpeeds.leftFront);
        leftBackDrive.setPower(combinedSpeeds.leftBack);
        rightFrontDrive.setPower(combinedSpeeds.rightFront);
        rightBackDrive.setPower(combinedSpeeds.rightBack);

        leftFrontPower = combinedSpeeds.leftFront;
        rightFrontPower = combinedSpeeds.righttFront;
        leftBackPower = combinedSpeeds.leftBack;
        rightBackPower = combinedSpeeds.rightBack;

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left front (%.2f), right front (%.2f), left back (%.2f), right back (%.2f)", leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

}

// base: /FtcRobotController\src\main\java\org\firstinspires\ftc\robotcontroller\external\samples\BasicOpMode_Iterative.java

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants.BasicBot;

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

@TeleOp(name="BasicBot: Tank Drive", group="Tank Drive")
public class BasicBot_TankDrive extends OpMode
{
    // Declare OpMode members.
    public DcMotor  leftFrontDrive;
    public DcMotor  leftBackDrive;
    public DcMotor  rightFrontDrive;
    public DcMotor  rightBackDrive;

    public static final double kLeftDeadZoneY = .1;
    public static final double kRightDeadZoneY = .1;

    private ElapsedTime runtime = new ElapsedTime();

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
        /*leftBackDrive  = hwMap.get(DcMotor.class, "left_back");
        rightFrontDrive = hwMap.get(DcMotor.class, "right_front");
        rightBackDrive = hwMap.get(DcMotor.class, "right_back");*/

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        /*leftBackDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD); // Set to FORWARD if using AndyMark motors
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);*/ // Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        leftFrontDrive.setPower(0);
        /*leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);*/

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        /*leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);*/
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
        double leftPower;
        double rightPower;

        //Get the values of the joysticks in this loop. A very very large number of loops happen every second
        double leftY = gamepad1.left_stick_y;
        double rightY  =  gamepad1.right_stick_y;

        //If sticks somehow are over 1/under -1 clip to those (max motor levels)
        leftPower = Range.clip(leftY, -1.0, 1.0);
        rightPower = Range.clip(rightY, -1.0, 1.0);

        //If the sticks are within a "deadzone" defined in Constants.java (.1 range initially) set them to 0
        if(Math.abs(leftPower) <= kLeftDeadZoneY){
            leftPower = 0;
        }
        if(Math.abs(rightPower) <= kRightDeadZoneY){
            rightPower = 0;
        }

        //Send the values to the motors
        leftFrontDrive.setPower(leftPower);
        /*leftBackDrive.setPower(leftPower);
        RightFrontDrive.setPower(rightPower);
        RightBackDrive.setPower(rightPower);*/

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        leftFrontDrive.setPower(0);
        /*leftBackDrive.setPower(0);
        RightFrontDrive.setPower(0);
        RightBackDrive.setPower(0);*/
    }

}
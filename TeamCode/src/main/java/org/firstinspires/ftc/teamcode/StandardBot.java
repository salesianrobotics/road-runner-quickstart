/********* SILVER EAGLE 2022-2023 FTC POWERPLAY **************/

/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * In this case that robot is a StandardBot.
 */
public abstract class StandardBot extends LinearOpMode {
    static final double TICKS_PER_MOTOR_REV = 537.7; // 5203-2402-0019 goBilda Motor Encoder
    //static final double PPR_ARM_MOTOR = 3895.9; // 5202 Series Yellow Jacket Planetary Gear Motor (139:1 Ratio, 43 RPM, 3.3 - 5V Encoder)

    static final double TICKS_PER_LINEAR_SLIDE_REVOLUTION = 28 * 5 * 5 ; //UltraPlanetary Gearbox HD Hex Motor x 5:1 gear-cartridge x 5:1 gear-cartridge

    static final double PULLEY_DIAMETER_INCHES = 1.27;
    static final double TICKS_PER_INCH_LINEAR_SLIDE = TICKS_PER_LINEAR_SLIDE_REVOLUTION/
            (PULLEY_DIAMETER_INCHES * Math.PI);

    static final double WHEEL_DIAMETER_INCHES = 4.00; // 5202-0002-0139 goBilda Motor Encoder...Diameter of wheel in inches

    static final double DRIVE_TRAIN_GEAR_RATIO = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_INCHES * Math.PI;
    static final double TICKS_PER_INCH = (TICKS_PER_MOTOR_REV * DRIVE_TRAIN_GEAR_RATIO) /
            WHEEL_CIRCUMFERENCE;

    static final double OPTIMAL_DRIVE_SPEED = 0.80 * StandardBot.MAX_DRIVE_TRAIN_VELOCITY;
    static final double AUTO_OPTIMAL_DRIVE_SPEED = .60 * StandardBot.MAX_DRIVE_TRAIN_VELOCITY;

    static final double OPTIMAL_TURN_SPEED = 0.4 * StandardBot.MAX_DRIVE_TRAIN_VELOCITY;
    static final double OPTIMAL_STRAFE_SPEED = 1.0 * StandardBot.MAX_DRIVE_TRAIN_VELOCITY;

    static final double OPTIMAL_CURRENT_TOLERANCE = 5000.00;

    static final double    LINEAR_SLIDE_OPTIMAL_CURRENT_TOLERANCE = 5000.0;

    static final int    LINEAR_SLIDE_POSITION_TOLERANCE = 100;
    static final double OPTIMAL_LINEAR_SLIDE_SPEED = 20.5 * TICKS_PER_LINEAR_SLIDE_REVOLUTION;
    static final double OPTIMAL_LINEAR_SLIDE_POWER = 0.5;

    static final double OPTIMAL_DISTNACE_FROM_JUNCTION = 4.0; // in inches

    static final float MIN_DETECTION_CONFIDENCE = 0.55f;

    static final double TILE_SIZE = 23.25;
    // Originally TILE_SIZW = 24,
    // however, Fr. Paul realized that the interlocking nature of the tiles made it .75 inch shorter

    static final double DRIVE_ADJUSTMENT = 0.01;
    static final double RIGHT_TURN_DEGREE_ADJUSTMENT = 0.3;
    static final double LEFT_TURN_DEGREE_ADJUSTMENT = 0.3;
    static final double LEFT_STRAFE_ADJUSTMENT = 0.15;
    static final double RIGHT_STRAFE_ADJUSTMENT = 0.30;
    static final double PERIMETER_X_ADJUSTMENT = 0.75; //inches less than our home field

    private final ElapsedTime period = new ElapsedTime();

    static final double TURN_COMPENSATION = 3.0;

    public DcMotorImplEx stdRightFront = null; // max velocity = 2840 ticks
    public static final int RIGHT_FRONT_MAX_VELOCITY = 2840;

    public DcMotorImplEx stdRightRear = null;  // max velocity = 2840 ticks
    public static final int RIGHT_REAR_MAX_VELOCITY = 2840;

    public DcMotorImplEx stdLeftFront = null; // max velocity = 2880 ticks
    public static final int LEFT_FRONT_MAX_VELOCITY = 2840;

    public DcMotorImplEx stdLeftRear = null;  // max velocity = 2840 ticks
    public static final int LEFT_REAR_MAX_VELOCITY = 2840;

    static final double MAX_DRIVE_TRAIN_VELOCITY = 2840; // min value of all 4 wheels above for drive speed consistency

    public DcMotorImplEx stdLinearSlide = null; // motor to control the linear slide
    public DistanceSensor stdDistanceSensorUnderGripper = null;
    public DistanceSensor stdFrontDistanceSensor = null;

    /* local OpMode members. */

    public Servo stdGripperServo = null;
    public static final double GRIPPER_CLOSED_POSITION = 0.70;
    public static final double GRIPPER_OPENED_POSITION = 0.50;
    public static final double GRIPPER_MIDDLE_POSITION = 0.55;
    //public static final double GRIPPER_NARROW_POSITION = 0.65;



    public Servo stdWristServo = null;
    static final double WRIST_UP_POSITION    = 0.50;     // Maximum rotational position
    static final double WRIST_DOWN_POSITION  = 0.05;     // Minimum rotational position
    static final double WRIST_MIDDLE_POSITION  = 0.35;//(WRIST_UP_POSITION + WRIST_DOWN_POSITION)/2.0 + 0.1;
    static final double WRIST_LOWER_GRAB_POSITION  = (WRIST_UP_POSITION + WRIST_DOWN_POSITION)/2.0;
    static final double WRIST_REST_POSITION = 0.70;

    static final double CONE_DIAMETER = 4.0;



    static final int HIGH_JUNCTION_POSITION = (int)(33.0 * TICKS_PER_INCH_LINEAR_SLIDE);
    static final int SLIGHTLY_BELOW_HIGH_JUNCTION_POSITION = (int)(32.0 * TICKS_PER_INCH_LINEAR_SLIDE);
    static final int MEDIUM_JUNCTION_POSITION = (int)(23.3 * TICKS_PER_INCH_LINEAR_SLIDE);
    static final int LOW_JUNCTION_POSITION = (int)(13.3 * TICKS_PER_INCH_LINEAR_SLIDE);
    static final int GROUND_JUNCTION_POSITION = (int)(1.0 * TICKS_PER_INCH_LINEAR_SLIDE);

    static final int GROUND_POSITION = (int)(0.0 * TICKS_PER_INCH_LINEAR_SLIDE);

    static final int AUTO_CONE_POSITION = (int)(5.2 * TICKS_PER_INCH_LINEAR_SLIDE);
    static final int AUTO_CONE_POSITION_DECREMENT = (int)(1.0 * TICKS_PER_INCH_LINEAR_SLIDE);


    static final int SLIGHT_DOWN_SLIDE_HIGH = (int)(28.0 * TICKS_PER_INCH_LINEAR_SLIDE);

    static final double CONE_STACK_DISTANCE_TOLERANCE =  0.5; // inches
    static final double CONE_STACK_WALL_DISTANCE_TOLERANCE = 1.0; // inches

    static final double OPTIMAL_DISTANCE_ROBOT_FROM_CONE_STACK_WALL = 4.00; // inches
    static final double OPTIMAL_DISTANCE_GRIPPER_FROM_GROUND = 6.5;

    /*
    static final int HIGH_JUNCTION_POSITION = (int)(7 * TICKS_PER_LINEAR_SLIDE_REVOLUTION);
    static final int MEDIUM_JUNCTION_POSITION = (int)(HIGH_JUNCTION_POSITION - 10 * TICKS_PER_INCH_LINEAR_SLIDE );
    static final int LOW_JUNCTION_POSITION = (int)(MEDIUM_JUNCTION_POSITION - 10 * TICKS_PER_INCH_LINEAR_SLIDE );
    static final int GROUND_JUNCTION_POSITION = (int)(0.47 * TICKS_PER_INCH_LINEAR_SLIDE);
    */

    public DcMotorImplEx stdArmMotor = null;
    HardwareMap hwMap = null;

    public SampleMecanumDrive stdMecanumDrive = null;

    /* Constructor */
    public StandardBot() {
    }

    public void checkMotorCurrent(String motorName, DcMotorEx motor, double maxCurrent)
    {
        telemetry.addData(motorName, "current is %5.2f milli-amps", motor.getCurrent(CurrentUnit.MILLIAMPS));

        if(motor.getCurrent(CurrentUnit.MILLIAMPS) > maxCurrent)
        {
            telemetry.addData(motorName, "current is over max tolerance of %5.2f milli-amps, stopping...", maxCurrent );
            motor.setVelocity(0.0);
        }

    }

    public void setDefaultMotorDirections() {
        stdLeftFront.setDirection(DcMotorEx.Direction.REVERSE);
        stdLeftRear.setDirection(DcMotorEx.Direction.REVERSE);
        stdRightFront.setDirection(DcMotorEx.Direction.FORWARD);
        stdRightRear.setDirection(DcMotorEx.Direction.FORWARD);

        stdLinearSlide.setDirection(DcMotorEx.Direction.FORWARD);

        //stdArmMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void setDriveTrainToRunWithoutEncoder() {
        stdLeftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        stdLeftRear.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        stdRightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        stdRightRear.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        //stdLinearSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetDriveTrainEncoder() {
        stdLeftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        stdLeftRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        stdRightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        stdRightRear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        //stdLinearSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setDriveTrainToRunUsingEncoder() {

        stdLeftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        stdLeftRear.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        stdRightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        stdRightRear.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        //stdLinearSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void setDriveTrainToBrakeOnZeroPower() {
        stdLeftRear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        stdLeftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        stdRightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        stdRightRear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        stdLinearSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void setAllMotorsToZeroPower() {

        // Set all motors to zero power
        stdLeftFront.setPower(0);
        stdLeftRear.setPower(0);
        stdRightFront.setPower(0);
        stdRightRear.setPower(0);
        stdLinearSlide.setPower(0);
        //stdArmMotor.setPower(0);
    }

    public void setDriveTrainToRunToPosition() {

        stdLeftFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        stdLeftRear.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        stdRightFront.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        stdRightRear.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        //stdLinearSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    public void setDriveTrainTargetPosition(int position) {
        stdLeftFront.setTargetPosition(position);
        stdLeftRear.setTargetPosition(position);
        stdRightFront.setTargetPosition(position);
        stdRightRear.setTargetPosition(position);
    }

    public void setDriveTrainVelocity(double velocity){
        stdLeftFront.setVelocity(velocity);
        stdLeftRear.setVelocity(velocity);
        stdRightFront.setVelocity(velocity);
        stdRightRear.setVelocity(velocity);
    }

    public void setCustomPIDFCoefficients(){
        // Instructions from FIRST GLOBAL Motor PIDF Tuning Guide (Updated 3 September 2019)

        // Set Custom Velocity PIDF Coefficients

        // stdRightFront's max velocity = 2840 ticks
        // stdRightRear's max velocity = 2880 ticks
        // stdLeftFront's max velocity = 2880 ticks
        // stdLeftRear's max velocity = 2840 ticks

        // F (driveTrain) = 32767 / 2840 ticks (RightFront & LeftRear)
        double F = 12.888;

        // P = 0.1 x F
        //double P = 0.1 * F;
        double P = 35.0;

        // I = 0.1 x P
        //double I = 0.1 * P;
        double I = 5.0;

        // D = 0
        // double D = 0.0;
        double D = 10.0;

        stdRightFront.setVelocityPIDFCoefficients(P, I, D, F);
        stdLeftRear.setVelocityPIDFCoefficients(P, I, D, F);
/*
        // F (driveTrain) = 32767 / 2880 ticks (RightRear & LeftFront)
        F = 11.3774305556;

        // P = 0.1 x F
        P = 0.1 * F;

        // I = 0.1 x P
        I = 0.1 * P;

        // D = 0
        D = 0.0;
*/
        stdRightRear.setVelocityPIDFCoefficients(P, I, D, F);
        stdLeftFront.setVelocityPIDFCoefficients(P, I, D, F);

        // Set Custom Position PIDF Coefficients
        // Regardless of your maximum velocity, you can set the position PIDF values to:
        // see FIRST GLOBAL Motor PIDF Tuning Guide (Updated 3 September 2019)

        stdRightFront.setPositionPIDFCoefficients(5.0);
        stdRightRear.setPositionPIDFCoefficients(5.0);
        stdLeftFront.setPositionPIDFCoefficients(5.0);
        stdLeftRear.setPositionPIDFCoefficients(5.0);
    }

    public boolean isDriveTrainBusy() {
        return stdLeftFront.isBusy() || stdLeftRear.isBusy()
                || stdRightFront.isBusy() || stdRightRear.isBusy();
    }

    public void turnRight() {
        encoderDrive(OPTIMAL_TURN_SPEED, TILE_SIZE, -TILE_SIZE);
    }

    public void turnRight(double angleInDegrees) {

        angleInDegrees -= RIGHT_TURN_DEGREE_ADJUSTMENT;

        double turnDistance = angleInDegrees/90.0 * (TILE_SIZE - TURN_COMPENSATION);
        encoderDrive(OPTIMAL_TURN_SPEED, turnDistance, -turnDistance);
    }

    public void turnLeft() {
        encoderDrive(OPTIMAL_TURN_SPEED, -TILE_SIZE, TILE_SIZE);
    }

    public void turnLeft(double angleInDegrees) {

        angleInDegrees += LEFT_TURN_DEGREE_ADJUSTMENT;

        double turnDistance = angleInDegrees/90.0 * (TILE_SIZE - TURN_COMPENSATION);
        encoderDrive(OPTIMAL_TURN_SPEED, -turnDistance, turnDistance);
    }


    public void moveForward(double nTiles) {
        nTiles += DRIVE_ADJUSTMENT;
        encoderDrive(AUTO_OPTIMAL_DRIVE_SPEED, nTiles * TILE_SIZE, nTiles * TILE_SIZE);
    }

    public void moveForward(double nTiles, double percentMaxSpeed) {
        nTiles += DRIVE_ADJUSTMENT;
        encoderDrive(percentMaxSpeed * MAX_DRIVE_TRAIN_VELOCITY, nTiles * TILE_SIZE, nTiles * TILE_SIZE);
    }

    public void moveBackward(double nTiles) {
        moveForward(-nTiles);
    }

    public void moveBackward(double nTiles, double percentMaxSpeed) {
        moveForward(-nTiles, percentMaxSpeed);
    }

    public void leftStrafe(double nTiles) {
        nTiles += LEFT_STRAFE_ADJUSTMENT; // custom adjustment for silver eagles

        encoderLeftStrafe(OPTIMAL_STRAFE_SPEED, nTiles * TILE_SIZE, nTiles * TILE_SIZE);
    }

    public void leftStrafe(double nTiles, double percentMaxSpeed) {

        nTiles += LEFT_STRAFE_ADJUSTMENT; // custom adjustment for silver eagles


        encoderLeftStrafe(percentMaxSpeed * OPTIMAL_STRAFE_SPEED, nTiles * TILE_SIZE, nTiles * TILE_SIZE);
    }

    public void rightStrafe(double nTiles) {

        nTiles += RIGHT_STRAFE_ADJUSTMENT;

        leftStrafe(-nTiles);
    }

    public void rightStrafe(double nTiles, double percentMaxSpeed) {

        nTiles += RIGHT_STRAFE_ADJUSTMENT;

        leftStrafe(-nTiles, percentMaxSpeed);
    }

    /******* movement by inches */
    public void moveForwardInches(double inches) {
        //nTiles += DRIVE_ADJUSTMENT;
        encoderDrive(AUTO_OPTIMAL_DRIVE_SPEED, inches, inches);
    }

    public void moveForwardInches(double inches, double percentMaxSpeed) {
        //nTiles += DRIVE_ADJUSTMENT;
        encoderDrive(percentMaxSpeed * MAX_DRIVE_TRAIN_VELOCITY, inches, inches);
    }

    public void moveBackwardInches(double inches) {
        moveForwardInches(-inches);
    }

    public void moveBackwardInches(double inches, double percentMaxSpeed) {
        moveForwardInches(-inches, percentMaxSpeed);
    }

    public void leftStrafeInches(double inches) {
        //nTiles += LEFT_STRAFE_ADJUSTMENT; // custom adjustment for silver eagles

        encoderLeftStrafe(OPTIMAL_STRAFE_SPEED, inches, inches);
    }

    public void leftStrafeInches(double inches, double percentMaxSpeed) {

        //nTiles += LEFT_STRAFE_ADJUSTMENT; // custom adjustment for silver eagles


        encoderLeftStrafe(percentMaxSpeed * OPTIMAL_STRAFE_SPEED, inches, inches);
    }

    public void rightStrafeInches(double inches) {

        //nTiles += RIGHT_STRAFE_ADJUSTMENT;

        leftStrafeInches(-inches);
    }

    public void rightStrafeInches(double inches, double percentMaxSpeed) {

        //nTiles += RIGHT_STRAFE_ADJUSTMENT;

        leftStrafeInches(-inches, percentMaxSpeed);
    }

    /* End movement by inches */

    public void setGripperPosition(double servoPosition)
    {
        stdGripperServo.setPosition(servoPosition);
    }

    public void setWristPosition(double servoPosition)
    {
        stdWristServo.setPosition(servoPosition);
    }

    public void encoderDrive(double speed, double leftInches, double rightInches)
    {
        ElapsedTime runtime = new ElapsedTime();

        int newLeftTarget;
        int newRightTarget;

        //telemetry.addData("encoderDrive", "starting...");

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            telemetry.addData("encoderDrive", "resetting drive train encoder");
            resetDriveTrainEncoder();

            // Determine new target position, and pass to motor controller
            newLeftTarget = stdLeftFront.getCurrentPosition() + (int) (leftInches * TICKS_PER_INCH);
            newRightTarget = stdRightFront.getCurrentPosition() + (int) (rightInches * TICKS_PER_INCH);

            telemetry.addData("encoderDrive", "newLeftTarget = %7d", newLeftTarget);
            stdLeftFront.setTargetPosition(newLeftTarget);
            stdLeftRear.setTargetPosition(newLeftTarget);

            telemetry.addData("encoderDrive", "newRightTarget = %7d", newRightTarget);
            stdRightFront.setTargetPosition(newRightTarget);
            stdRightRear.setTargetPosition(newRightTarget);

            telemetry.addData("encoderDrive", "setting drive train to RUN_TO_POSITION");
            setDriveTrainToRunToPosition();

            telemetry.addData("encoderDrive", "setting DriveTrainVelocity to %7.2f", speed);
            setDriveTrainVelocity(speed);

            //telemetry.update();


            // reset the timeout time and start motion.
            runtime.reset();
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the this will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && isDriveTrainBusy()
                    && !stdLeftFront.isOverCurrent() && !stdLeftRear.isOverCurrent()
                    && !stdRightFront.isOverCurrent() && !stdRightRear.isOverCurrent())
            {

                // Display it for the driver.
                telemetry.addData("encoderDrive", "target position left:%7d, right:%7d", newLeftTarget, newRightTarget);
                telemetry.addData("encoderDrive", "current position %7d :%7d, %7d :%7d",
                        stdLeftFront.getCurrentPosition(),
                        stdLeftRear.getCurrentPosition(),
                        stdRightFront.getCurrentPosition(),
                        stdRightRear.getCurrentPosition());
                telemetry.update();

                checkMotorCurrent("LeftFrontMotor", stdLeftFront, OPTIMAL_CURRENT_TOLERANCE);
                checkMotorCurrent("LeftRearMotor", stdLeftRear, OPTIMAL_CURRENT_TOLERANCE);
                checkMotorCurrent("RightFrontMotor", stdRightFront, OPTIMAL_CURRENT_TOLERANCE);
                checkMotorCurrent("RightReartMotor", stdRightRear, OPTIMAL_CURRENT_TOLERANCE);

            }

            // Stop all motion;
            setDriveTrainVelocity(0.0);

        }
    }

    public void encoderLeftStrafe(double speed, double leftInches, double rightInches) {
        ElapsedTime runtime = new ElapsedTime();
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            //telemetry.addData("encoderDrive", "Resetting Drive Train Encoder");
            resetDriveTrainEncoder();

            // Determine new target position, and pass to motor controller
            newLeftTarget = stdLeftFront.getCurrentPosition() + (int) (leftInches * TICKS_PER_INCH);
            newRightTarget = stdRightFront.getCurrentPosition() + (int) (rightInches * TICKS_PER_INCH);

            stdLeftFront.setTargetPosition(-newLeftTarget);
            stdLeftRear.setTargetPosition(newLeftTarget);
            stdRightFront.setTargetPosition(newRightTarget);
            stdRightRear.setTargetPosition(-newRightTarget);

            // Turn On RUN_TO_POSITION
            setDriveTrainToRunToPosition();

            // reset the timeout time and start motion.
            runtime.reset();

            setDriveTrainVelocity(speed);
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the this continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && isDriveTrainBusy()
                    && !stdLeftFront.isOverCurrent() && !stdLeftRear.isOverCurrent()
                    && !stdRightFront.isOverCurrent() && !stdRightRear.isOverCurrent())
            {
                // Display it for the driver.
                //telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                //telemetry.addData("Path2", "Running at %7d :%7d, %7d :%7d",
                //        stdLeftFront.getCurrentPosition(),
                //        stdLeftRear.getCurrentPosition(),
                //        stdRightFront.getCurrentPosition(),
                //        stdRightRear.getCurrentPosition());
                //telemetry.update();
            }
            // Stop all motion;
            setDriveTrainVelocity(0.0);
        }
    }


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors

        // enable default standard drive below only if SampleMechanumDrive is not used
        stdLeftFront = hwMap.get(DcMotorImplEx.class, "LeftFront");
        stdRightFront = hwMap.get(DcMotorImplEx.class, "RightFront");
        stdLeftRear = hwMap.get(DcMotorImplEx.class, "LeftRear");
        stdRightRear = hwMap.get(DcMotorImplEx.class, "RightRear");


        stdLinearSlide = hwMap.get(DcMotorImplEx.class, "LinearSlide");

        //stdArmMotor = hwMap.get(DcMotorImplEx.class, "Arm");

        // Define and initialize ALL installed servos.

        stdGripperServo = hwMap.get(Servo.class, "GripperServo");
        stdWristServo = hwMap.get(Servo.class, "WristServo");

        stdDistanceSensorUnderGripper = hwMap.get(DistanceSensor.class, "GripperSensor");
        stdFrontDistanceSensor = hwMap.get(DistanceSensor.class, "FrontDistanceSensor");

        resetDriveTrainEncoder();
        setDriveTrainToRunWithoutEncoder();
        setDefaultMotorDirections();

        setDriveTrainToBrakeOnZeroPower();
        setAllMotorsToZeroPower();
        //setCustomPIDFCoefficients();

        stdLeftFront.setCurrentAlert(StandardBot.OPTIMAL_CURRENT_TOLERANCE, CurrentUnit.MILLIAMPS);
        stdLeftRear.setCurrentAlert(StandardBot.OPTIMAL_CURRENT_TOLERANCE, CurrentUnit.MILLIAMPS);
        stdRightFront.setCurrentAlert(StandardBot.OPTIMAL_CURRENT_TOLERANCE, CurrentUnit.MILLIAMPS);
        stdRightRear.setCurrentAlert(StandardBot.OPTIMAL_CURRENT_TOLERANCE, CurrentUnit.MILLIAMPS);
        stdLinearSlide.setCurrentAlert(StandardBot.OPTIMAL_CURRENT_TOLERANCE, CurrentUnit.MILLIAMPS);

        stdWristServo.setPosition(WRIST_MIDDLE_POSITION);
        telemetry.addData("WristServo:", "MIDDLE POSITION");
        //stdGripperServo.setPosition(GRIPPER_MIDDLE_POSITION);




        //telemetry.addData("GripperServo", "position (%.2f)", stdGripperServo.getPosition());
        //telemetry.addData("WristServo", "position (%.2f)", stdWristServo.getPosition());

        telemetry.update();
    }

    public void initRoadRunner(HardwareMap ahwMap) {
        hwMap = ahwMap;

        stdMecanumDrive = new SampleMecanumDrive(ahwMap);
        stdLinearSlide = hwMap.get(DcMotorImplEx.class, "LinearSlide");

        // Define and initialize ALL installed servos.

        stdGripperServo = hwMap.get(Servo.class, "GripperServo");
        stdWristServo = hwMap.get(Servo.class, "WristServo");

        stdDistanceSensorUnderGripper = hwMap.get(DistanceSensor.class, "GripperSensor");
        stdFrontDistanceSensor = hwMap.get(DistanceSensor.class, "FrontDistanceSensor");

        stdLinearSlide.setCurrentAlert(StandardBot.OPTIMAL_CURRENT_TOLERANCE, CurrentUnit.MILLIAMPS);
        stdWristServo.setPosition(WRIST_MIDDLE_POSITION);

        telemetry.addData("WristServo:", "MIDDLE POSITION");
        telemetry.update();
    }

    public void raiseLinearSlide(int position) {


        //telemetry.addData("raiseLinearSlide", "Setting to RUN WITH ENCODER");
        stdLinearSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        stdLinearSlide.setDirection(DcMotorEx.Direction.FORWARD);

        //telemetry.addData("raiseLinearSlide", "Setting TargetPosition to %5d", position);
        stdLinearSlide.setTargetPosition(position);

        //telemetry.addData("raiseLinearSlide", "Setting Run to Position");
        stdLinearSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);  // Can't hurt to call this repeatedly

        //telemetry.addData("raiseLinearSlide", "Setting Arm Velocity to %7.2f", OPTIMAL_ARM_SPEED );
        stdLinearSlide.setVelocity(OPTIMAL_LINEAR_SLIDE_SPEED);

        while (stdLinearSlide.isBusy() && !stdLinearSlide.isOverCurrent())
        {
            sleep(10);
            //   telemetry.addData("raiseLinearSlide", "targetPosition is %7d", stdLinearSlide.getTargetPosition());
            //   telemetry.addData("raiseLinearSlide", "currentPosition is %7d", stdLinearSlide.getCurrentPosition());
            //   telemetry.addData("raiseLinearSlide", "CURRENT is %5.2f milli-amps", stdLinearSlide.getCurrent(CurrentUnit.MILLIAMPS));

            //   telemetry.update();
        }

        //telemetry.addData("raiseLinearSlide", "Setting velocity to 0.0 now");
        stdLinearSlide.setVelocity(0.0);

    }

    public void adjustLinearSlide(double targetDistanceFromGround) {

        //telemetry.addData("raiseLinearSlide", "Setting to RUN WITH ENCODER");
        stdLinearSlide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        stdLinearSlide.setDirection(DcMotorEx.Direction.FORWARD);

        while (!stdLinearSlide.isOverCurrent() &&
                Math.abs(stdDistanceSensorUnderGripper.getDistance(DistanceUnit.INCH) - targetDistanceFromGround) > CONE_STACK_DISTANCE_TOLERANCE)
        {
            if (stdDistanceSensorUnderGripper.getDistance(DistanceUnit.INCH) < targetDistanceFromGround) { //
                stdLinearSlide.setPower(OPTIMAL_LINEAR_SLIDE_POWER);
            }
            else {
                stdLinearSlide.setPower(-OPTIMAL_LINEAR_SLIDE_POWER);
            }

            sleep(10);
            telemetry.addData("adjustLinearSlide", "Distance is %5.2f", stdDistanceSensorUnderGripper.getDistance(DistanceUnit.INCH) );

            telemetry.addData("adjustLinearSlide", "Power is %5.2f", stdLinearSlide.getPower());
            telemetry.addData("adjustLinearSlide", "CURRENT is %5.2f milli-amps", stdLinearSlide.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.update();

        }
        stdLinearSlide.setPower(0.0);
    }

    public void adjustToConeStackWall(double targetDistance) {

        //telemetry.addData("raiseLinearSlide", "Setting to RUN WITH ENCODER");
        //setDriveTrainToRunWithoutEncoder();
        //setDefaultMotorDirections();

        while (Math.abs(getFrontDistance() - targetDistance) > CONE_STACK_WALL_DISTANCE_TOLERANCE)
        {
            if (getFrontDistance() > targetDistance) { //
                moveForwardInches(getFrontDistance() - targetDistance);
            }
            else {
                moveBackwardInches(targetDistance - getFrontDistance());
            }

            sleep(10);
            telemetry.addData("adjustLinearSlide", "Distance is %5.2f", stdDistanceSensorUnderGripper.getDistance(DistanceUnit.INCH) );

            telemetry.addData("adjustLinearSlide", "Power is %5.2f", stdLinearSlide.getPower());
            telemetry.addData("adjustLinearSlide", "CURRENT is %5.2f milli-amps", stdLinearSlide.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.update();

        }
        stdLinearSlide.setPower(0.0);
    }
    public void lowerLinearSlide(int position) {


        //telemetry.addData("raiseLinearSlide", "Setting to RUN WITH ENCODER");
        stdLinearSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        stdLinearSlide.setDirection(DcMotorEx.Direction.REVERSE);

        //telemetry.addData("raiseLinearSlide", "Setting TargetPosition to %5d", position);
        stdLinearSlide.setTargetPosition(position);

        //telemetry.addData("raiseLinearSlide", "Setting Run to Position");
        stdLinearSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);  // Can't hurt to call this repeatedly

        //telemetry.addData("raiseLinearSlide", "Setting Arm Velocity to %7.2f", OPTIMAL_ARM_SPEED );
        stdLinearSlide.setVelocity(OPTIMAL_LINEAR_SLIDE_SPEED);

        while (stdLinearSlide.isBusy() && !stdLinearSlide.isOverCurrent())
        {
            sleep(10);
            //   telemetry.addData("raiseLinearSlide", "targetPosition is %7d", stdLinearSlide.getTargetPosition());
            //   telemetry.addData("raiseLinearSlide", "currentPosition is %7d", stdLinearSlide.getCurrentPosition());
            //   telemetry.addData("raiseLinearSlide", "CURRENT is %5.2f milli-amps", stdLinearSlide.getCurrent(CurrentUnit.MILLIAMPS));

            //   telemetry.update();
        }

        //telemetry.addData("raiseLinearSlide", "Setting velocity to 0.0 now");
        stdLinearSlide.setVelocity(0.0);

    }

    public double getFrontDistance(){ // in INCHES
        return stdFrontDistanceSensor.getDistance(DistanceUnit.INCH);
    }

    public double getGripperDistanceFromGround() { // in INCHES
        return stdDistanceSensorUnderGripper.getDistance(DistanceUnit.INCH);
    }

    public void stopBeforeConeStackWall()
    {
        while (isDriveTrainBusy())
        {
            if (getFrontDistance() <= OPTIMAL_DISTANCE_ROBOT_FROM_CONE_STACK_WALL
                    + CONE_STACK_DISTANCE_TOLERANCE)
            {
                setDriveTrainVelocity(0.0);
            }
        }
    }

}
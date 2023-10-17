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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotMap {

    /* Public OpMode members. */
    public DcMotor drivetrainLeft = null;
    public DcMotor drivetrainRight = null;
    public DcMotor drivetrainCenter = null;
    public DcMotor elevatorLeftMotor = null;
    public DcMotor elevatorRightMotor = null;

    public Servo grabberLeftServo = null;
    public Servo grabberRightServo = null;
    public Servo hookLeftServo = null;
    public Servo hookRightServo = null;
    public Servo deployServo = null;

    public static final double GRABBER_GRAB = 0.52;
    public static final double GRABBER_CAPSTONE = 0.43;
    public static final double GRABBER_RELEASE = 1.0;
    public static final double HOOK_GRAB = 0.75;
    public static final double HOOK_RELEASE = 0.05;
    public static final double DEPLOY_START = 0.0;
    public static final double DEPLOY_DEPLOY = 1.0;

    public static final int TICKS_PER_REV = 1120; //40:1 HD HEX MOTOR
    public static final int WHEEL_DIAMETER = 90; //REV ROBOTICS 90 MM OMNI-WHEEL
    public static final double MILLIMETER_TO_INCHES_CONSTANT = 25.4;
    public static final double WHEEL_CIRCUMFERENCE_INCHES = (Math.PI * WHEEL_DIAMETER) / MILLIMETER_TO_INCHES_CONSTANT; // ~ 11.13 in. IN 1120 ticks (1 Rev)
    public static final double COUNTS_PER_INCH = TICKS_PER_REV / WHEEL_CIRCUMFERENCE_INCHES; //DRIVETRAIN's COUNTS PER INCH

    public static final double MAX_AUTO_COURSE_POWER = 1.0;
    public static final double MAX_AUTO_STRAFE_POWER = 0.5;
    public static final double MAX_AUTO_ROTATE_POWER = 0.75;

    /* local OpMode members. */
    HardwareMap hardwareMap = null;
    private ElapsedTime runtime = new ElapsedTime();

    /* Constructor */
    public RobotMap() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hardwareMap = ahwMap;

        drivetrainLeft = hardwareMap.get(DcMotor.class, "drivetrainLeft");
        drivetrainRight = hardwareMap.get(DcMotor.class, "drivetrainRight");
        drivetrainCenter = hardwareMap.get(DcMotor.class, "drivetrainCenter");
        elevatorLeftMotor = hardwareMap.get(DcMotor.class, "elevatorLeftMotor");
        elevatorRightMotor = hardwareMap.get(DcMotor.class, "elevatorRightMotor");

        grabberLeftServo = hardwareMap.get(Servo.class, "grabberLeftServo");
        grabberRightServo = hardwareMap.get(Servo.class, "grabberRightServo");
        hookLeftServo = hardwareMap.get(Servo.class,"hookLeftServo");
        hookRightServo = hardwareMap.get(Servo.class,"hookRightServo");
        deployServo = hardwareMap.get(Servo.class,"deployServo");

        // Reverse the motor that runs backwards when connected directly to the battery
        drivetrainLeft.setDirection(DcMotor.Direction.REVERSE);
        drivetrainRight.setDirection(DcMotor.Direction.FORWARD);
        drivetrainCenter.setDirection(DcMotor.Direction.FORWARD);
        elevatorLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        elevatorRightMotor.setDirection(DcMotor.Direction.FORWARD);

        grabberLeftServo.setDirection(Servo.Direction.FORWARD);
        grabberRightServo.setDirection(Servo.Direction.REVERSE);
        hookLeftServo.setDirection(Servo.Direction.FORWARD);
        hookRightServo.setDirection(Servo.Direction.REVERSE);

        // Reset motor encoders
        drivetrainLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drivetrainCenter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drivetrainRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set motor default run mode to use encoders
        drivetrainLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drivetrainCenter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drivetrainRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevatorRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevatorLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public int encoderConvert(double inches) {
        int units = (int) Math.round(COUNTS_PER_INCH * inches); //The RUT_TO_POSITION function of DcMotor requires an int to be passed in. We cast the answer of this line down to an int to satisfy this.
        return units;
    }

    public void singleMotorRunToDistance(DcMotor motor, double distanceInches, double timeoutSeconds) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(encoderConvert(distanceInches));
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        runtime.startTime();

        while (runtime.seconds() <= timeoutSeconds && motor.isBusy()) {
            //This waits until the timeout has ended or the motor is at the position, probably a better way to write this but it works.
            if (runtime.seconds() <= 1.0) {
                motor.setPower(0.25);
            }
            if (runtime.seconds() > 1.0 && runtime.seconds() < 1.5) {
                motor.setPower(0.3);
            }

            if (runtime.seconds() >= 1.5 && runtime.seconds() <= 2.5) {
                motor.setPower(0.35);
            }

            if (runtime.seconds() > 2.5 && runtime.seconds() < 3.5) {
                motor.setPower(0.35);
            }

            if (runtime.seconds() >= 3.5 && runtime.seconds() <= 4.5) {
                motor.setPower(0.35);
            }

            if (runtime.seconds() > 4.5) {
                motor.setPower(0.35);
            }
        }

        motor.setPower(0.0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void dualMotorRunToDistance(DcMotor motor1, DcMotor motor2, double motor1DistanceInches, double motor2DistanceInches, double powerPercent, double timeoutSeconds) {
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor1.setTargetPosition(encoderConvert(motor1DistanceInches));
        motor2.setTargetPosition(encoderConvert(motor2DistanceInches));

        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        runtime.startTime();
        motor1.setPower(powerPercent);
        motor2.setPower(powerPercent);

        while (runtime.seconds() <= timeoutSeconds && motor1.isBusy() && motor2.isBusy()) {
            //This waits until the timeout has ended or the motors are at the position, probably a better way to write this but it works.
        }

        motor1.setPower(0.0);
        motor2.setPower(0.0);

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}

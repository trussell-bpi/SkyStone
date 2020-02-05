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

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import pkg3939.Robot3939;
import pkg3939.skystoneDetectorClass;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="AutoTemplate2", group="Pushbot")
//@Disabled
public class AutoTemplate2 extends LinearOpMode {

    /* Declare OpMode members. */
    Robot3939 robot = new Robot3939();   // Use a Pushbot's hardware

    skystoneDetectorClass detector = new skystoneDetectorClass();
    int[] vals;
    private ElapsedTime     runtime = new ElapsedTime();

    private DistanceSensor sensorRange;
    private final double gearRatio = 2/1;//2:1
    private final double ticksPerRev = 383.6 * gearRatio;
    private final double wheelCircumference = 3.1415 * robot.wheelDiameter; //pi * diameter (inches)

    public void rotate(double power, double time) {
        robot.FL.setPower(power);
        robot.FR.setPower(-power);
        robot.RL.setPower(power);
        robot.RR.setPower(-power);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 4:  Stop and close the claw.
        robot.FL.setPower(0);
        robot.FR.setPower(0);
        robot.RL.setPower(0);
        robot.RR.setPower(0);
        sleep(200);

    }

    public void rotateEnc(int targetTicks) {
        double k = 0.0004;

        robot.stopAndResetEncoders();

        robot.useEncoders(true);

        double tickAvg = robot.FL.getCurrentPosition()/4;
        double tickDifference = targetTicks - tickAvg;
        double power = Range.clip(k*Math.abs(tickDifference) + 0.2, 0.2, 1);

        if(tickDifference < 0)
            power = -power;

        if(opModeIsActive()) {
            robot.RL.setTargetPosition(targetTicks);
            robot.RR.setTargetPosition(-targetTicks);
            robot.FL.setTargetPosition(targetTicks);
            robot.FR.setTargetPosition(-targetTicks);

            robot.RUN_TO_POSITION();

            robot.setAllGivenPower(power);

            double startAngle = robot.getAngle();

            while(robot.RL.isBusy() || robot.RR.isBusy() || robot.FL.isBusy() || robot.FR.isBusy()) {
                //wait till motor finishes

                tickAvg = (int)((Math.abs(robot.FL.getCurrentPosition()) + Math.abs(robot.FR.getCurrentPosition()) + Math.abs(robot.RL.getCurrentPosition()) + Math.abs(robot.RR.getCurrentPosition()))/4);
                tickDifference = targetTicks - tickAvg;
                power = Range.clip(k*Math.abs(tickDifference) + 0.2, 0.2, 1);

                if(tickDifference < 0)
                    power = -power;
//                 double correction = robot.getCorrection(startAngle, Math.abs(power));//check if someone is pushing you
//                robot.FL.setPower(power - correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
//                robot.FR.setPower(power + correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
//                robot.RR.setPower(power + correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
//                robot.RL.setPower(power - correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)

                robot.setAllGivenPower(power);

                telemetry.addData("Path", "Driving "+" inches");
                telemetry.addData("tickDifference", tickDifference);
                telemetry.addData("power", power);
                telemetry.update();
            }

            robot.stopMotors();

            telemetry.addData("Path", "Complete");
            telemetry.update();

            robot.useEncoders(true);
        }
    }

    public void runToAngle(int targetAngle) {
        double k = 0.004;
        double angleDifference = targetAngle - robot.getAngle();
        double power = Range.clip(k*Math.abs(angleDifference) + 0.2, 0.2, 1);

        if(angleDifference < 0)
            power = -power;

        robot.useEncoders(false);

        robot.FL.setPower(power);
        robot.FR.setPower(-power);
        robot.RL.setPower(power);
        robot.RR.setPower(-power);

        boolean run = true;

        runtime.reset();
        while (opModeIsActive() && run) {
            angleDifference = targetAngle - robot.getAngle();
            power = Range.clip(k*Math.abs(angleDifference) + 0.2, 0.2, 1);
            if(angleDifference < 0)
                power = -power;

            robot.FL.setPower(power);
            robot.FR.setPower(-power);
            robot.RL.setPower(power);
            robot.RR.setPower(-power);
            telemetry.addData("angle difference", angleDifference);
            telemetry.addData("power", robot.FL.getPower());
            telemetry.update();
            if(Math.round(robot.getAngle()) == targetAngle)
                run = false;
            if(runtime.seconds() > 2)
                run = false;
        }

        // Step 4:  Stop and close the claw.
        robot.FL.setPower(0);
        robot.FR.setPower(0);
        robot.RL.setPower(0);
        robot.RR.setPower(0);
        sleep(200);

        robot.useEncoders(true);
    }

    public void rotateAngle(double power, double angle, double threshold) {
        angle = -angle;
        if(angle < 0)
            power = -power;

        robot.useEncoders(false);

        robot.FL.setPower(power);
        robot.FR.setPower(-power);
        robot.RL.setPower(power);
        robot.RR.setPower(-power);

        double newAngle = robot.getAngle() + angle;
        boolean run = true;

        runtime.reset();
        while (opModeIsActive() && run) {
            if(Math.abs(robot.getAngle() - newAngle) < threshold)
                run = false;
            if(runtime.seconds() > 4)
                run = false;
        }

        // Step 4:  Stop and close the claw.
        robot.FL.setPower(0);
        robot.FR.setPower(0);
        robot.RL.setPower(0);
        robot.RR.setPower(0);
        sleep(200);

        robot.useEncoders(true);

    }

    public void strafe(double power, double time) {
        robot.FL.setPower(power);
        robot.FR.setPower(-power);
        robot.RL.setPower(-power);
        robot.RR.setPower(power);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 4:  Stop and close the claw.
        robot.FL.setPower(0);
        robot.FR.setPower(0);
        robot.RL.setPower(0);
        robot.RR.setPower(0);
        mySleep(0.2);

    }

    public void strafeGyro(double power, double time) {//uses gyro to make sure robot's angle stays the same - prevents unintentional rotation
        robot.useEncoders(false);
        double startAngle = robot.getAngle();
        robot.FLpower = +power;
        robot.FRpower = -power;
        robot.RRpower = +power;
        robot.RLpower = -power;

        robot.setAllPower();

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {//we get on the train together
            double correction = robot.getCorrection(startAngle, Math.abs(power));//check if someone is pushing you
            robot.FL.setPower(+power - correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
            robot.FR.setPower(-power + correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
            robot.RR.setPower(+power + correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
            robot.RL.setPower(-power - correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
        }
        robot.setAllGivenPower(0);//once we're at the station, forget anything happened
        mySleep(0.2);

    }

    public void moveDistanceGyro(double power, double time) {
        double startAngle = robot.getAngle();
        robot.FL.setPower(-power);
        robot.FR.setPower(-power);
        robot.RL.setPower(-power);
        robot.RR.setPower(-power);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {//we get on the train together
            double correction = robot.getCorrection(startAngle, Math.abs(power));//check if someone is pushing you
            robot.FL.setPower(-power - correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
            robot.FR.setPower(-power + correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
            robot.RR.setPower(-power + correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
            robot.RL.setPower(-power - correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        robot.setAllGivenPower(0);
        mySleep(0.2);

    }

    public void mySleep(double time) {//seconds
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {

        }
    }

    public void moveDistance(double power, double time) {

        robot.FL.setPower(power);
        robot.FR.setPower(power);
        robot.RL.setPower(power);
        robot.RR.setPower(power);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 4:  Stop and close the claw.
        robot.FL.setPower(0);
        robot.FR.setPower(0);
        robot.RL.setPower(0);
        robot.RR.setPower(0);
        sleep(200);

    }

    public void moveEncoderDifferential(double distance) {
        double k = 0.0005;
        double minSpeed = 0.2;
        double startSpeed = 0.5;
        double rotations = distance/ wheelCircumference; //distance / circumference (inches)
        int targetTicks = (int)(rotations*ticksPerRev);
        int currentTickAvg = (robot.FL.getCurrentPosition() + robot.FR.getCurrentPosition() + robot.RL.getCurrentPosition() + robot.RR.getCurrentPosition())/4;
        double tickDifference = targetTicks - currentTickAvg;
        double power = Range.clip(k*Math.abs(tickDifference) + minSpeed, minSpeed, startSpeed);
        double failsafe = 1/26*Math.abs(distance) + 3;

        robot.stopAndResetEncoders();
        robot.useEncoders(true);

        if(opModeIsActive()) {
            robot.RL.setTargetPosition(targetTicks);
            robot.RR.setTargetPosition(targetTicks);
            robot.FL.setTargetPosition(targetTicks);
            robot.FR.setTargetPosition(targetTicks);

            robot.RUN_TO_POSITION();

            robot.setAllGivenPower(power);//sets all motors to the given power

            double startAngle = robot.getAngle();
            runtime.reset();

            while(robot.RL.isBusy() || robot.RR.isBusy() || robot.FL.isBusy() || robot.FR.isBusy()) {
                //wait till motor finishes

                if(runtime.seconds() > failsafe)//fail safe, in case of infinite loop
                    break;

                currentTickAvg = (int)((robot.FL.getCurrentPosition() + robot.FR.getCurrentPosition() + robot.RL.getCurrentPosition() + robot.RR.getCurrentPosition())/4.0);
                tickDifference = targetTicks - currentTickAvg;

                if(runtime.seconds() < 0.15)
                    power = startSpeed;
                else
                    power = Range.clip(k*Math.abs(tickDifference) + minSpeed, minSpeed, 1);

                double correction = robot.getCorrection(startAngle, Math.abs(power));//check if someone is pushing you
                robot.FL.setPower(Range.clip(power - correction, -1, 1));//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
                robot.FR.setPower(Range.clip(power + correction, -1, 1));//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
                robot.RR.setPower(Range.clip(power + correction, -1, 1));//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
                robot.RL.setPower(Range.clip(power - correction, -1, 1));//if so, push him/her back to defend your seat(correction), but the train keeps going(power)

                telemetry.addData("Path", "Driving "+distance+" inches");
                telemetry.addData("target ticks", targetTicks);
                telemetry.addData("tick Avg", currentTickAvg);
                telemetry.addData("tickDifference", tickDifference);
                telemetry.addData("power", power);
                telemetry.addData("FL ticks", robot.FL.getCurrentPosition());
                telemetry.addData("FR ticks", robot.FR.getCurrentPosition());
                telemetry.addData("RR ticks", robot.RR.getCurrentPosition());
                telemetry.addData("RL ticks", robot.RL.getCurrentPosition());
                telemetry.addData("timer", (int)runtime.seconds());
                telemetry.update();
            }
            robot.stopMotors();
            robot.useEncoders(true);

            telemetry.addData("Path", "Complete");
            telemetry.update();

        }
    }

    public void moveDistanceEnc(double power, double distance) {
        double rotations = distance/ wheelCircumference; //distance / circumference (inches)
        int targetTicks = (int)(rotations*ticksPerRev);
        robot.stopAndResetEncoders();
        robot.useEncoders(true);

        if(opModeIsActive()) {
            robot.setAllTargetPosition(targetTicks);
            robot.RUN_TO_POSITION();
            robot.setAllGivenPower(power);
            double startAngle = robot.getAngle();
            while(robot.RL.isBusy() || robot.RR.isBusy() || robot.FL.isBusy() || robot.FR.isBusy()) {
                //wait till motor finishes working
                double correction = robot.getCorrection(startAngle, Math.abs(power));//check if someone is pushing you
                robot.FL.setPower(power - correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
                robot.FR.setPower(power + correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
                robot.RR.setPower(power + correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
                robot.RL.setPower(power - correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
                telemetry.addData("Path", "Driving "+distance+" inches");
                telemetry.update();
            }

            robot.stopMotors();
            telemetry.addData("Path", "Complete");
            telemetry.update();
        }
    }

    public void strafeEnc(double power, double distance) {
        double rotations = distance/ wheelCircumference; //distance / circumference (inches)
        int targetTicks = (int)(rotations*ticksPerRev);

        robot.stopAndResetEncoders();
        robot.useEncoders(true);

        if(opModeIsActive()) {
            robot.RL.setTargetPosition(-targetTicks);
            robot.RR.setTargetPosition(targetTicks);
            robot.FL.setTargetPosition(targetTicks);
            robot.FR.setTargetPosition(-targetTicks);

            robot.RUN_TO_POSITION();

            robot.setAllGivenPower(power);

            double startAngle = robot.getAngle();

            while(robot.RL.isBusy() || robot.RR.isBusy() || robot.FL.isBusy() || robot.FR.isBusy()) {
                //wait till motor finishes working
                double correction = robot.getCorrection(startAngle, Math.abs(power));//check if someone is pushing you
                robot.FL.setPower(Range.clip(power - correction, -1, 1));//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
                robot.FR.setPower(Range.clip(power + correction, -1, 1));//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
                robot.RR.setPower(Range.clip(power + correction, -1, 1));//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
                robot.RL.setPower(Range.clip(power - correction, -1, 1));//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
                telemetry.addData("Path", "Driving "+distance+" inches");
                telemetry.update();
            }

            robot.stopMotors();

            telemetry.addData("Path", "Complete");
            telemetry.update();

            robot.useEncoders(true);
        }
    }

    public void strafeEncoderDifferential(double distance) {
        robot.stopAndResetEncoders();
        robot.useEncoders(true);

        double k = 0.0005;
        double minSpeed = 0.2;
        double startSpeed = 0.6;
        double rotations = distance/ wheelCircumference; //distance / circumference (inches)
        int targetTicks = (int)(rotations*ticksPerRev);
        double tickAvg = robot.RR.getCurrentPosition();
        double tickDifference = targetTicks - tickAvg;
        double power = Range.clip(k*Math.abs(tickDifference) + minSpeed, minSpeed, startSpeed);

        if(opModeIsActive()) {
            robot.RL.setTargetPosition(-targetTicks);
            robot.RR.setTargetPosition(targetTicks);
            robot.FL.setTargetPosition(targetTicks);
            robot.FR.setTargetPosition(-targetTicks);

            robot.RUN_TO_POSITION();

            robot.setAllGivenPower(power);

            double startAngle = robot.getAngle();
            runtime.reset();

            while(robot.RL.isBusy() || robot.RR.isBusy() || robot.FL.isBusy() || robot.FR.isBusy()) {
                //wait till motor finishes

                if(runtime.seconds() > 5)//fail safe, in case of infinite loop
                    break;

                tickAvg = robot.RR.getCurrentPosition();
                tickDifference = targetTicks - tickAvg;

                if(Math.abs(tickDifference) < 10)
                    break;

                if(runtime.seconds() < 0.3)
                    power = startSpeed;
                else
                    power = Range.clip(k*Math.abs(tickDifference) + minSpeed, minSpeed, 1);

                robot.setAllGivenPower(power);


                double correction = robot.getCorrection(startAngle, Math.abs(power));//check if someone is pushing you
                robot.FL.setPower(Range.clip(power - correction, -1, 1));//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
                robot.FR.setPower(Range.clip(power + correction, -1, 1));//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
                robot.RR.setPower(Range.clip(power + correction, -1, 1));//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
                robot.RL.setPower(Range.clip(power - correction, -1, 1));//if so, push him/her back to defend your seat(correction), but the train keeps going(power)

                telemetry.addData("Path", "Driving "+distance+" inches");
                telemetry.addData("ti ckDifference", tickDifference);
                telemetry.addData("FL ticks", robot.FL.getCurrentPosition());
                telemetry.addData("FR ticks", robot.FR.getCurrentPosition());
                telemetry.addData("RR ticks", robot.RR.getCurrentPosition());
                telemetry.addData("RL ticks", robot.RL.getCurrentPosition());
                telemetry.addData("power", power);
                telemetry.update();
            }

            robot.stopMotors();
            mySleep(0.5);

            telemetry.addData("Path", "Complete");
            telemetry.update();
        }
    }

    @Override
    public void runOpMode() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.initMotors(hardwareMap);
        robot.initServos(hardwareMap);//servo
        robot.initIMU(hardwareMap);//gyro
        robot.useEncoders(true);

        detector.setOffset(0f/8f, 1.7f/8f);
        detector.camSetup(hardwareMap);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();
        while (!isStopRequested() && !robot.imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

//        telemetry.addData("Mode", "waiting for start");
//        telemetry.addData("imu calib status", robot.imu.getCalibrationStatus().toString());
        // you can use this as a regular DistanceSensor.
        //sensorRange = hardwareMap.get(DistanceSensor.class, "2Msensor1");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
//        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;

        telemetry.addData(">>", "Press start to continue");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        if(opModeIsActive()) {
//            detector.updateVals();
//            vals = detector.getVals();
//            telemetry.addData("Values", vals[1] + "   " + vals[0] + "   " + vals[2]);
//            telemetry.addLine("hi");
//            telemetry.update();
//            telemetry.addData("deviceName",sensorRange.getDeviceName() );
//            telemetry.addData("range", String.format("%.01f mm", sensorRange.getDistance(DistanceUnit.MM)));
//            telemetry.addData("range", String.format("%.01f cm", sensorRange.getDistance(DistanceUnit.CM)));
//            telemetry.addData("range", String.format("%.01f m", sensorRange.getDistance(DistanceUnit.METER)));
//            telemetry.addData("range", String.format("%.01f in", sensorRange.getDistance(DistanceUnit.INCH)));

//            moveEncoderDifferential(32);
//            moveEncoderDifferential(-10 );
//            mySleep(3);
//            moveDistanceEnc(1, 32);
//            moveDistanceEnc(1, -32);
//            mySleep(3);
//            strafeGyro(-1, 3);
//            strafeGyro(1, 3);

//            strafeEnc(1, -30);
//            strafeEnc(1, 30);
//            strafeEnc(1, -30);
//            strafeEnc(1, 30);
//            mySleep(3);
//            strafeEncoderDifferential(-30);
//            strafeEncoderDifferential(30);
//            strafeEncoderDifferential(-30);
//            strafeEncoderDifferential(30);

            strafeEncoderDifferential(-38);


            telemetry.update();

        }
    }
}

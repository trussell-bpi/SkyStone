//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//
//import pkg3939.Robot3939;
//import pkg3939.skystoneDetectorClass;
//
//public class nicksTest extends LinearOpMode {
//    /* Declare OpMode members. */
//    Robot3939 robot = new Robot3939();   // Use a Pushbot's hardware
//
//    skystoneDetectorClass detector = new skystoneDetectorClass();
//    int[] vals;
//
//    private ElapsedTime runtime = new ElapsedTime();
//
//    private final double gearRatio = 2/1;//2:1
//    private final double ticksPerRev = 383.6 * gearRatio;
//    private final double wheelCircumference = 3.1415 * robot.wheelDiameter; //pi * diameter (inches)
//
//    public void rotate(double power, double time) {
//        robot.FL.setPower(power);
//        robot.FR.setPower(-power);
//        robot.RL.setPower(power);
//        robot.RR.setPower(-power);
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < time)) {
//            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }
//
//        // Step 4:  Stop and close the claw.
//        robot.FL.setPower(0);
//        robot.FR.setPower(0);
//        robot.RL.setPower(0);
//        robot.RR.setPower(0);
//        sleep(200);
//
//    }
//
//    public void rotateEnc(int targetTicks, double failsafe) {
//        double k = 0.0006;
//
//        robot.stopAndResetEncoders();
//
//        robot.useEncoders(true);
//
//        double tickAvg = robot.FL.getCurrentPosition()/4;
//        double tickDifference = targetTicks - tickAvg;
//        double power = Range.clip(k*Math.abs(tickDifference) + 0.2, 0.2, 1);
//
//        if(tickDifference < 0)
//            power = -power;
//
//        if(opModeIsActive()) {
//            robot.RL.setTargetPosition(targetTicks);
//            robot.RR.setTargetPosition(-targetTicks);
//            robot.FL.setTargetPosition(targetTicks);
//            robot.FR.setTargetPosition(-targetTicks);
//
//            robot.RL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.RR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            robot.setAllGivenPower(power);
//
//            double startAngle = robot.getAngle();
//            runtime.reset();
//
//            while(robot.RL.isBusy() || robot.RR.isBusy() || robot.FL.isBusy() || robot.FR.isBusy()) {
//                //wait till motor finishes
////                if(runtime.seconds() > failsafe) {//fail safe, in case of infinite loop
////                    robot.stopMotors();
////                    break;
////                }
//                if(runtime.seconds() > failsafe)//fail safe, in case of infinite loop
//                    break;
//
//
//                tickAvg = (int)((Math.abs(robot.FL.getCurrentPosition()) + Math.abs(robot.FR.getCurrentPosition()) + Math.abs(robot.RL.getCurrentPosition()) + Math.abs(robot.RR.getCurrentPosition()))/4);
//                tickDifference = targetTicks - tickAvg;
//                power = Range.clip(k*Math.abs(tickDifference) + 0.2, 0.2, 1);
//
//                if(Math.abs(tickDifference) < 15)
//                    break;
//
////                 double correction = robot.getCorrection(startAngle, Math.abs(power));//check if someone is pushing you
////                robot.FL.setPower(power - correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
////                robot.FR.setPower(power + correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
////                robot.RR.setPower(power + correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
////                robot.RL.setPower(power - correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
//
//                robot.setAllGivenPower(power);
//
//                telemetry.addData("Path", "Driving "+" inches");
//                telemetry.addData("tickDifference", tickDifference);
//                telemetry.addData("power", power);
//                telemetry.update();
//            }
//
//            robot.stopMotors();
//
//            telemetry.addData("Path", "Complete");
//            telemetry.update();
//
//            robot.useEncoders(true);
//        }
//    }
//
//    public void runToAngle(int targetAngle) {
//        double k = 0.004;
//        double angleDifference = targetAngle - robot.getAngle();
//        double power = Range.clip(k*Math.abs(angleDifference) + 0.2, 0.2, 1);
//
//        if(angleDifference < 0)
//            power = -power;
//
//        robot.useEncoders(false);
//
//        robot.FL.setPower(power);
//        robot.FR.setPower(-power);
//        robot.RL.setPower(power);
//        robot.RR.setPower(-power);
//
//        boolean run = true;
//
//        runtime.reset();
//        while (opModeIsActive() && run) {
//            angleDifference = targetAngle - robot.getAngle();
//            power = Range.clip(k*Math.abs(angleDifference) + 0.2, 0.2, 1);
//            if(angleDifference < 0)
//                power = -power;
//
//            robot.FL.setPower(power);
//            robot.FR.setPower(-power);
//            robot.RL.setPower(power);
//            robot.RR.setPower(-power);
//            telemetry.addData("angle difference", angleDifference);
//            telemetry.addData("power", robot.FL.getPower());
//            telemetry.update();
//            if(Math.round(robot.getAngle()) == targetAngle)
//                run = false;
//            if(runtime.seconds() > 2)
//                run = false;
//        }
//
//        // Step 4:  Stop and close the claw.
//        robot.FL.setPower(0);
//        robot.FR.setPower(0);
//        robot.RL.setPower(0);
//        robot.RR.setPower(0);
//        sleep(200);
//
//        robot.useEncoders(true);
//    }
//
//    public void rotateAngle(double power, double angle, double threshold) {
//        angle = -angle;
//        if(angle < 0)
//            power = -power;
//
//        robot.useEncoders(false);
//
//        robot.FL.setPower(power);
//        robot.FR.setPower(-power);
//        robot.RL.setPower(power);
//        robot.RR.setPower(-power);
//
//        double newAngle = robot.getAngle() + angle;
//        boolean run = true;
//
//        runtime.reset();
//        while (opModeIsActive() && run) {
//            if(Math.abs(robot.getAngle() - newAngle) < threshold)
//                run = false;
//            if(runtime.seconds() > 4)
//                run = false;
//        }
//
//        // Step 4:  Stop and close the claw.
//        robot.FL.setPower(0);
//        robot.FR.setPower(0);
//        robot.RL.setPower(0);
//        robot.RR.setPower(0);
//        sleep(200);
//
//        robot.useEncoders(true);
//
//    }
//
//    public void strafe(double power, double time) {
//        robot.FL.setPower(power);
//        robot.FR.setPower(-power);
//        robot.RL.setPower(-power);
//        robot.RR.setPower(power);
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < time)) {
//            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }
//
//        // Step 4:  Stop and close the claw.
//        robot.FL.setPower(0);
//        robot.FR.setPower(0);
//        robot.RL.setPower(0);
//        robot.RR.setPower(0);
//        mySleep(0.2);
//
//    }
//
//    private void strafeGyro(double power, double time) {//uses gyro to make sure robot's angle stays the same - prevents unintentional rotation
//        double startAngle = robot.getAngle();
//        robot.FLpower = +power;
//        robot.FRpower = -power;
//        robot.RRpower = +power;
//        robot.RLpower = -power;
//
//        robot.setAllPower();
//        robot.useEncoders(true);
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < time)) {//we get on the train together
//            double correction = robot.getCorrection(startAngle, Math.abs(power));//check if someone is pushing you
//            robot.FL.setPower(+power - correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
//            robot.FR.setPower(-power + correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
//            robot.RR.setPower(+power + correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
//            robot.RL.setPower(-power - correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
//        }
//        robot.setAllGivenPower(0);//once we're at the station, forget anything happened
//
//    }
//
//    public void strafeGyroWhileExtending(double power, double time) {
//
//    }
//
//    public void moveDistanceGyro(double power, double time) {
//        double startAngle = robot.getAngle();
//        robot.FL.setPower(power);
//        robot.FR.setPower(power);
//        robot.RL.setPower(power);
//        robot.RR.setPower(power);
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < time)) {//we get on the train together
//            double correction = robot.getCorrection(startAngle, Math.abs(power));//check if someone is pushing you
//            robot.FL.setPower(power + correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
//            robot.FR.setPower(power - correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
//            robot.RR.setPower(power - correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
//            robot.RL.setPower(power + correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
//            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }
//
//        robot.setAllGivenPower(0);
//        mySleep(0.2);
//    }
//
//    public void mySleep(double time) {//seconds
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < time)) {
//
//        }
//    }
//
//    public void moveDistance(double power, double time) {
//
//        robot.FL.setPower(power);
//        robot.FR.setPower(power);
//        robot.RL.setPower(power);
//        robot.RR.setPower(power);
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < time)) {
//            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }
//
//        // Step 4:  Stop and close the claw.
//        robot.FL.setPower(0);
//        robot.FR.setPower(0);
//        robot.RL.setPower(0);
//        robot.RR.setPower(0);
//        sleep(200);
//
//    }
//
//    public void moveEncoderDifferential(double distance, double failsafe) {
//        double k = 0.0005;
//        double minSpeed = 0.2;
//        double startSpeed = 0.5;
//        double rotations = distance/ wheelCircumference; //distance / circumference (inches)
//        int targetTicks = (int)(rotations*ticksPerRev);
//        int currentTickAvg = (robot.FL.getCurrentPosition() + robot.FR.getCurrentPosition() + robot.RL.getCurrentPosition() + robot.RR.getCurrentPosition())/4;
//        double tickDifference = targetTicks - currentTickAvg;
//        double power = Range.clip(k*Math.abs(tickDifference) + minSpeed, minSpeed, startSpeed);
//
////        if(tickDifference < 0)
////            power = -power;
//
//        robot.stopAndResetEncoders();
//        robot.useEncoders(true);
//
//        if(opModeIsActive()) {
//            robot.RL.setTargetPosition(targetTicks);
//            robot.RR.setTargetPosition(targetTicks);
//            robot.FL.setTargetPosition(targetTicks);
//            robot.FR.setTargetPosition(targetTicks);
//
//            robot.RUN_TO_POSITION();
//
//            robot.setAllGivenPower(power);//sets all motors to the given power
//
//            double startAngle = robot.getAngle();
//            runtime.reset();
//
//            while(robot.RL.isBusy() || robot.RR.isBusy() || robot.FL.isBusy() || robot.FR.isBusy()) {
//                //wait till motor finishes
//                robot.hinge.setPosition(robot.hinge.getPosition());
//
//                if(runtime.seconds() > failsafe)//fail safe, in case of infinite loop
//                    break;
//
//
//                currentTickAvg = (int)((robot.FL.getCurrentPosition() + robot.FR.getCurrentPosition() + robot.RL.getCurrentPosition() + robot.RR.getCurrentPosition())/4.0);
//                tickDifference = targetTicks - currentTickAvg;
//
//                if(Math.abs(tickDifference) < 10)
//                    break;
//
//                if(runtime.seconds() < 0.3)
//                    power = startSpeed;
//                else
//                    power = Range.clip(k*Math.abs(tickDifference) + minSpeed, minSpeed, 1);
//
////                if(tickDifference < 0)
////                    power = -power;
////                 double correction = robot.getCorrection(startAngle, Math.abs(power));//check if someone is pushing you
////                robot.FL.setPower(power - correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
////                robot.FR.setPower(power + correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
////                robot.RR.setPower(power + correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
////                robot.RL.setPower(power - correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
//
//                robot.setAllGivenPower(power);
//
//                telemetry.addData("target ticks", targetTicks);
//                telemetry.addData("tick Avg", currentTickAvg);
//                telemetry.addData("tickDifference", tickDifference);
//                telemetry.addData("power", power);
//                telemetry.addData("stonearm pos", robot.stoneArm.getPosition());
//                telemetry.update();
//            }
//            robot.stopMotors();
//
//            telemetry.addData("Path", "Complete");
//            telemetry.update();
//
//        }
//    }
//
//    public void moveEncoderDifferentialWhileExtendingSlides(double distance, int slideTicks) {
//        double k = 0.0005;
//        double minSpeed = 0.2;
//        double startSpeed = 0.5;
//        double rotations = distance/ wheelCircumference; //distance / circumference (inches)
//        int targetTicks = (int)(rotations*ticksPerRev);
//        int currentTickAvg = (robot.FL.getCurrentPosition() + robot.FR.getCurrentPosition() + robot.RL.getCurrentPosition() + robot.RR.getCurrentPosition())/4;
//        double tickDifference = targetTicks - currentTickAvg;
//        double power = Range.clip(k*Math.abs(tickDifference) + minSpeed, minSpeed, startSpeed);
//
//        robot.stopAndResetEncoders();
//        robot.useEncoders(true);
//
//        if(opModeIsActive()) {
//            robot.RL.setTargetPosition(targetTicks);
//            robot.RR.setTargetPosition(targetTicks);
//            robot.FL.setTargetPosition(targetTicks);
//            robot.FR.setTargetPosition(targetTicks);
//
//            //all run to position
//            robot.RUN_TO_POSITION();
//            robot.leftSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            robot.rightSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//
//            robot.setAllGivenPower(power);//sets all motors to the given power
//            robot.leftSlides.setPower(-1);
//            robot.rightSlides.setPower(-1);
//
//
//            double startAngle = robot.getAngle();
//            boolean hold = true;
//            runtime.reset();
//
//            while(robot.RL.isBusy() || robot.RR.isBusy() || robot.FL.isBusy() || robot.FR.isBusy() || robot.leftSlides.isBusy() || robot.rightSlides.isBusy()) {
//                //wait till motor finishes
//                if(runtime.seconds() > 1.2 && hold) {
//                    robot.leftSlides.setPower(0);
//                    robot.rightSlides.setPower(0);
//
//                    robot.hinge.setPosition(0.04);
//
//                    robot.leftSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    robot.rightSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//                    hold = false;
//                }
//
//                if(runtime.seconds() > 5)//fail safe, in case of infinite loop
//                    break;
//
//
//                currentTickAvg = (int)((robot.FL.getCurrentPosition() + robot.FR.getCurrentPosition() + robot.RL.getCurrentPosition() + robot.RR.getCurrentPosition())/4.0);
//                tickDifference = targetTicks - currentTickAvg;
//
//                if(runtime.seconds() < 0.3)
//                    power = startSpeed;
//                else
//                    power = Range.clip(k*Math.abs(tickDifference) + minSpeed, minSpeed, 1);
//
////                if(tickDifference < 0)
////                    power = -power;
////                 double correction = robot.getCorrection(startAngle, Math.abs(power));//check if someone is pushing you
////                robot.FL.setPower(power - correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
////                robot.FR.setPower(power + correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
////                robot.RR.setPower(power + correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
////                robot.RL.setPower(power - correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
//
//                robot.setAllGivenPower(power);
//
//                telemetry.addData("Path", "Driving "+distance+" inches");
//                telemetry.addData("target ticks", targetTicks);
//                telemetry.addData("tick Avg", currentTickAvg);
//                telemetry.addData("tickDifference", tickDifference);
//                telemetry.addData("power", power);
//                telemetry.addData("FL ticks", robot.FL.getCurrentPosition());
//                telemetry.addData("FR ticks", robot.FR.getCurrentPosition());
//                telemetry.addData("RR ticks", robot.RR.getCurrentPosition());
//                telemetry.addData("RL ticks", robot.RL.getCurrentPosition());
//                telemetry.addData("timer", (int)runtime.seconds());
//                telemetry.update();
//            }
//
//            robot.leftSlides.setPower(0);
//            robot.rightSlides.setPower(0);
//
//            robot.leftSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            robot.rightSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//            robot.leftSlides.setPower(-0.3);
//            robot.rightSlides.setPower(-0.3);
//
//            robot.stopMotors();
//            robot.useEncoders(true);
//
//            telemetry.addData("Path", "Complete");
//            telemetry.update();
//
//        }
//    }
//
//    public void moveDistanceEnc(double power, double distance) {
//        robot.stopAndResetEncoders();
//
//        robot.useEncoders(true);
//
//        double rotations = distance/ wheelCircumference; //distance / circumference (inches)
//        int targetTicks = (int)(rotations*ticksPerRev);
//
//        if(opModeIsActive()) {
//            robot.RL.setTargetPosition(targetTicks);
//            robot.RR.setTargetPosition(targetTicks);
//            robot.FL.setTargetPosition(targetTicks);
//            robot.FR.setTargetPosition(targetTicks);
//
//            robot.RL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.RR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            robot.setAllGivenPower(power);
//
//            double startAngle = robot.getAngle();
//
//            while(robot.RL.isBusy() || robot.RR.isBusy() || robot.FL.isBusy() || robot.FR.isBusy()) {
//                //wait till motor finishes working
//                double correction = robot.getCorrection(startAngle, Math.abs(power));//check if someone is pushing you
//                robot.FL.setPower(power - correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
//                robot.FR.setPower(power + correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
//                robot.RR.setPower(power + correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
//                robot.RL.setPower(power - correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
//                telemetry.addData("Path", "Driving "+distance+" inches");
//                telemetry.update();
//            }
//
//            robot.stopMotors();
//
//            telemetry.addData("Path", "Complete");
//            telemetry.update();
//
//            robot.useEncoders(true);
//        }
//    }
//
//    public void strafeEnc(double power, double distance) {
//        robot.stopAndResetEncoders();
//        robot.useEncoders(true);
//
//        double rotations = distance/ wheelCircumference; //distance / circumference (inches)
//        int targetTicks = (int)(rotations*ticksPerRev);
//
//        if(opModeIsActive()) {
//            robot.RL.setTargetPosition(-targetTicks);
//            robot.RR.setTargetPosition(targetTicks);
//            robot.FL.setTargetPosition(targetTicks);
//            robot.FR.setTargetPosition(-targetTicks);
//
//            robot.RUN_TO_POSITION();
//
//            robot.setAllGivenPower(power);
//
//            double startAngle = robot.getAngle();
//
//            while(robot.RL.isBusy() || robot.RR.isBusy() || robot.FL.isBusy() || robot.FR.isBusy()) {
//                //wait till motor finishes working
//                double correction = robot.getCorrection(startAngle, Math.abs(power));//check if someone is pushing you
//                robot.FL.setPower(Range.clip(power - correction, -1, 1));//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
//                robot.FR.setPower(Range.clip(power + correction, -1, 1));//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
//                robot.RR.setPower(Range.clip(power + correction, -1, 1));//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
//                robot.RL.setPower(Range.clip(power - correction, -1, 1));//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
//                telemetry.addData("Path", "Driving "+distance+" inches");
//                telemetry.update();
//            }
//
//            robot.stopMotors();
//
//            telemetry.addData("Path", "Complete");
//            telemetry.update();
//
//            robot.useEncoders(true);
//        }
//    }
//
//    public void strafeEncoderDifferential(double distance) {
//        robot.stopAndResetEncoders();
//        robot.useEncoders(true);
//
//        double k = 0.0006;
//        double minSpeed = 0.2;
//        double startSpeed = 0.6;
//        double rotations = distance/ wheelCircumference; //distance / circumference (inches)
//        int targetTicks = (int)(rotations*ticksPerRev);
//        double tickAvg = robot.RR.getCurrentPosition();
//        double tickDifference = targetTicks - tickAvg;
//        double power = Range.clip(k*Math.abs(tickDifference) + minSpeed, minSpeed, startSpeed);
//
//        if(opModeIsActive()) {
//            robot.RL.setTargetPosition(-targetTicks);
//            robot.RR.setTargetPosition(targetTicks);
//            robot.FL.setTargetPosition(targetTicks);
//            robot.FR.setTargetPosition(-targetTicks);
//
//            robot.RUN_TO_POSITION();
//
//            robot.setAllGivenPower(power);
//
//            double startAngle = robot.getAngle();
//            runtime.reset();
//
//            while(robot.RL.isBusy() || robot.RR.isBusy() || robot.FL.isBusy() || robot.FR.isBusy()) {
//                //wait till motor finishes
//
//                if(runtime.seconds() > 3)//fail safe, in case of infinite loop
//                    break;
//
//                tickAvg = robot.RR.getCurrentPosition();
//                tickDifference = targetTicks - tickAvg;
//
//                if(Math.abs(tickDifference) < 15)
//                    break;
//
//                if(runtime.seconds() < 0.3)
//                    power = startSpeed;
//                else
//                    power = Range.clip(k*Math.abs(tickDifference) + minSpeed, minSpeed, 1);
//
//                robot.setAllGivenPower(power);
//
//
//                double correction = robot.getCorrection(startAngle, Math.abs(power));//check if someone is pushing you
//                robot.FL.setPower(Range.clip(power - correction, -1, 1));//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
//                robot.FR.setPower(Range.clip(power + correction, -1, 1));//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
//                robot.RR.setPower(Range.clip(power + correction, -1, 1));//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
//                robot.RL.setPower(Range.clip(power - correction, -1, 1));//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
//
//                telemetry.addData("Path", "Driving "+distance+" inches");
//                telemetry.addData("tickDifference", tickDifference);
//                telemetry.addData("power", power);
//                telemetry.update();
//            }
//
//            robot.stopMotors();
//
//            telemetry.addData("Path", "Complete");
//            telemetry.update();
//        }
//    }
//
//    public void strafeEncoderDifferential(double distance) {
//        double k = 0.0004;
//        double minSpeed = 0.2;
//        double startSpeed = 0.6;
//
//        robot.stopAndResetEncoders();
//        robot.useEncoders(true);
//
//        double rotations = distance/ wheelCircumference; //distance / circumference (inches)
//        int targetTicks = (int)(rotations*ticksPerRev);
//
//        double tickAvg = (int)((robot.FL.getCurrentPosition() + robot.FR.getCurrentPosition() + robot.RL.getCurrentPosition() + robot.RR.getCurrentPosition())/4);
//        double tickDifference = targetTicks - tickAvg;
//        double power = Range.clip(k*Math.abs(tickDifference) + minSpeed, minSpeed, startSpeed);
//
//        if(tickDifference < 0)
//            power = -power;
//
//        if(opModeIsActive()) {
//            robot.RL.setTargetPosition(-targetTicks);
//            robot.RR.setTargetPosition(targetTicks);
//            robot.FL.setTargetPosition(targetTicks);
//            robot.FR.setTargetPosition(-targetTicks);
//
//            robot.RL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.RR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            robot.setAllGivenPower(power);
//
//            double startAngle = robot.getAngle();
//
//            while(robot.RL.isBusy() || robot.RR.isBusy() || robot.FL.isBusy() || robot.FR.isBusy()) {
//                //wait till motor finishes
//
//                if(runtime.seconds() > 5)//fail safe, in case of infinite loop
//                    break;
//
//
//                tickAvg = (int)((Math.abs(robot.FL.getCurrentPosition()) + Math.abs(robot.FR.getCurrentPosition()) + Math.abs(robot.RL.getCurrentPosition()) + Math.abs(robot.RR.getCurrentPosition()))/4.0);
//                tickDifference = targetTicks - tickAvg;
//
//                if(runtime.seconds() < 0.3)
//                    power = startSpeed;
//                else
//                    power = Range.clip(k*Math.abs(tickDifference) + minSpeed, minSpeed, 1);
//
//
//
////                 double correction = robot.getCorrection(startAngle, Math.abs(power));//check if someone is pushing you
////                robot.FL.setPower(power - correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
////                robot.FR.setPower(power + correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
////                robot.RR.setPower(power + correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
////                robot.RL.setPower(power - correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
//
//                robot.setAllGivenPower(power);
//
//                telemetry.addData("Path", "Driving "+distance+" inches");
//                telemetry.addData("tickDifference", tickDifference);
//                telemetry.addData("power", power);
//                telemetry.update();
//            }
//
//            robot.stopMotors();
//            mySleep(0.5);
//
//            telemetry.addData("Path", "Complete");
//            telemetry.update();
//
//            robot.useEncoders(true);
//        }
//    }
//
//    public void moveSlides(double power, int constant) {
//        if (opModeIsActive()) {
//            robot.leftSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.rightSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            robot.leftSlides.setTargetPosition(robot.leftSlides.getCurrentPosition() + constant);
//            robot.rightSlides.setTargetPosition(robot.rightSlides.getCurrentPosition() + constant);
//
//            robot.leftSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.rightSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            robot.leftSlides.setPower(power);
//            robot.rightSlides.setPower(power);
//
//            runtime.reset();
//
//            while (robot.leftSlides.isBusy() || robot.rightSlides.isBusy()) {
//                //wait till motor finishes working
//                robot.drive(gamepad1.left_stick_x,
//                        gamepad1.left_stick_y,
//                        gamepad1.right_stick_x);
//                telemetry.addLine("Slides Extending");
//                telemetry.update();
//                if (runtime.seconds() > 1.2)
//                    break;
//            }
//            telemetry.addLine("Extended");
//            telemetry.update();
//
//            robot.leftSlides.setPower(0);
//            robot.rightSlides.setPower(0);
//
//            robot.leftSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            robot.rightSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//            robot.leftSlides.setPower(-0.25);
//            robot.rightSlides.setPower(-0.25);
//            //
////            robot.leftSlides.setPower(0);
////            robot.rightSlides.setPower(0);
//        }
//    }
//
//    public void moveDistanceGyroWhileExtending(double power, double time)   {
//        double startAngle = robot.getAngle();
//        robot.FL.setPower(power);
//        robot.FR.setPower(power);
//        robot.RL.setPower(power);
//        robot.RR.setPower(power);
//        runtime.reset();
//        robot.setSlidesPower(-1);
//        while (opModeIsActive() && (runtime.seconds() < time)) {//we get on the train together
//            double correction = robot.getCorrection(startAngle, Math.abs(power));//check if someone is pushing you
//            double extendTime = Math.round(runtime.milliseconds());
//            if(550 > extendTime && extendTime < 450) {
//                robot.setSlidesPower(-0.3);
//                robot.hinge.setPosition(0.72);
//                mySleep(0.4);
//                robot.stoneArm.setPosition(0.26);
//                mySleep(0.5);
//                robot.leftSlides.setPower(0);
//                robot.rightSlides.setPower(0);
//            }
//            robot.FL.setPower(power + correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
//            robot.FR.setPower(power - correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
//            robot.RR.setPower(power - correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
//            robot.RL.setPower(power + correction);//if so, push him/her back to defend your seat(correction), but the train keeps going(power)
//            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
//            telemetry.update();
//        }
//
//        robot.setAllGivenPower(0);
//    }
//
//    public void runOpMode() {
//
//    }
//}

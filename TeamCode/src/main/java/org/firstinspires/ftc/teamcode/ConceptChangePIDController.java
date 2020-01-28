package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import pkg3939.Robot3939;

/**
 * Created by tom on 9/26/17.
 * This assumes that you are using a REV Robotics Expansion Hub
 * as your DC motor controller. This op mode uses the extended/enhanced
 * PID-related functions of the DcMotorControllerEx class.
 * The REV Robotics Expansion Hub supports the extended motor controller
 * functions, but other controllers (such as the Modern Robotics and
 * Hitechnic DC Motor Controllers) do not.
 */

@Autonomous(name="Concept: Change PID Controller", group = "Examples")
public class ConceptChangePIDController extends LinearOpMode {

    // our DC motor.
    Robot3939 robot = new Robot3939();

    public static final double NEW_P = 2.5;
    public static final double NEW_I = 0.1;
    public static final double NEW_D = 0.2;
    public static final double NEW_F = 0.0;


    public void runOpMode() {
        // get reference to DC motor.
        robot.initMotors(hardwareMap);

        // wait for start command.
        waitForStart();

        // get a reference to the motor controller and cast it as an extended functionality controller.
        // we assume it's a REV Robotics Expansion Hub (which supports the extended controller functions).
        DcMotorControllerEx ExRL = (DcMotorControllerEx)robot.RL.getController();
        DcMotorControllerEx ExRR = (DcMotorControllerEx)robot.RR.getController();
        DcMotorControllerEx ExFL = (DcMotorControllerEx)robot.FL.getController();
        DcMotorControllerEx ExFR = (DcMotorControllerEx)robot.FR.getController();

        // get the port number of our configured motor.
        int indexRL = ((DcMotorEx)robot.RL).getPortNumber();
        int indexRR = ((DcMotorEx)robot.RR).getPortNumber();
        int indexFR = ((DcMotorEx)robot.FR).getPortNumber();
        int indexFL = ((DcMotorEx)robot.FL).getPortNumber();

//
//        // get the PID coefficients for the RUN_USING_ENCODER  modes.
//        PIDFCoefficients pidOrig = ExRL.getPIDFCoefficients(motorIndex, DcMotor.RunMode.RUN_USING_ENCODER);

        // change coefficients.
        PIDFCoefficients pidNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        ExRL.setPIDFCoefficients(indexRL, DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
        ExRR.setPIDFCoefficients(indexRR, DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
        ExFR.setPIDFCoefficients(indexFR, DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
        ExFL.setPIDFCoefficients(indexFL, DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
//
//        // re-read coefficients and verify change.
        PIDFCoefficients modRL = ExRL.getPIDFCoefficients(indexRL, DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients modRR = ExRR.getPIDFCoefficients(indexRR, DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients modFR = ExFR.getPIDFCoefficients(indexFR, DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients modFL = ExFL.getPIDFCoefficients(indexFL, DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Runtime", "%.03f", getRuntime());
        telemetry.addData("P,I,D (modified)", "%.04f, %.04f, %.04f",
                modRL.p, modRL.i, modRL.d);
        telemetry.update();


        // display info to user.
        if(opModeIsActive()) {
            ExRL.setMotorMode(indexRL, DcMotor.RunMode.RUN_USING_ENCODER);
            ExRR.setMotorMode(indexRR, DcMotor.RunMode.RUN_USING_ENCODER);
            ExFL.setMotorMode(indexFL, DcMotor.RunMode.RUN_USING_ENCODER);
            ExFR.setMotorMode(indexFR, DcMotor.RunMode.RUN_USING_ENCODER);

            ExRL.setMotorTargetPosition(indexRL, 1000);
            ExRR.setMotorTargetPosition(indexRR, 1000);
            ExFL.setMotorTargetPosition(indexFL, 1000);
            ExFR.setMotorTargetPosition(indexFR, 1000);

            ExRL.setMotorMode(indexRL, DcMotor.RunMode.RUN_TO_POSITION);
            ExRR.setMotorMode(indexRR, DcMotor.RunMode.RUN_TO_POSITION);
            ExFL.setMotorMode(indexFL, DcMotor.RunMode.RUN_TO_POSITION);
            ExFR.setMotorMode(indexFR, DcMotor.RunMode.RUN_TO_POSITION);

            ExRL.setMotorPower(indexRL, 1);
            ExRR.setMotorPower(indexRR, 1);
            ExFL.setMotorPower(indexFL, 1);
            ExFR.setMotorPower(indexFR, 1);

            while(ExRL.isBusy(indexRL) || ExRR.isMotorEnabled(indexRR) || ExFL.isBusy(indexFL) || ExFR.isMotorEnabled(indexFR)) {

            }



            telemetry.addData("Runtime", "%.03f", getRuntime());
            telemetry.addData("P,I,D (modified)", "%.04f, %.04f, %.04f",
                    modRL.p, modRL.i, modRL.d);
            telemetry.update();
        }
    }

}
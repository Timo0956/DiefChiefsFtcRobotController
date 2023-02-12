package org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Tele.untested.newTeleOp.HoriLinSlide.testHorizontalLinSlide;
import org.firstinspires.ftc.teamcode.Tele.untested.servoStuff.ServoTele;

//import org.firstinspires.ftc.teamcode.Tele.tested.initialize2023;


@TeleOp

public class OpWithModeIntegration extends LinearOpMode {
    @Override

    public void runOpMode() throws InterruptedException{

        //Linear slides
        DcMotor rightLinSlide = hardwareMap.dcMotor.get("rightLinSlide"); //Right vertical linear slide motor
        DcMotor leftLinSlide = hardwareMap.dcMotor.get("leftLinSlide"); //Left vertical linear slide motor
        DcMotor HL = hardwareMap.dcMotor.get("Horizontal LinSlide"); //Motor for horizontal linear slide

        //Wheels
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft"); //Motor for front left wheel
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft"); //Motor for back left wheel
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight"); //Motor for front right wheel
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight"); //Motor for back right wheel

        //Motor/CRServo for top arm
        DcMotor TopMotor = hardwareMap.dcMotor.get("TopMotor"); //Motor for spinning blue cone picker upper around the robot (front to back)
        CRServo ClawServo = hardwareMap.crservo.get("clawServo"); //CRServo to spin blue cone picker upper thing

        //Servos for intake claw
        Servo ClawServoL = hardwareMap.servo.get("clawServoL"); //Left side intake claw servo
        Servo ClawServoR = hardwareMap.servo.get("clawServoR"); //Right side intake claw servo

        //Servos for forearm
        Servo servo3A = hardwareMap.servo.get("3a"); //Forearm - 3a
        Servo servo2A = hardwareMap.servo.get("2a"); //Forearm - 2a

        //Initializing hardware
        clawServoClass.clawServoInit(ClawServo);  //Initializes CRServo for blue cone picky uppy
        dualServoForearm.initForearmServos(servo3A, servo2A); //Initializes Servos for forearm
        ServoTele.setServos(ClawServoL, ClawServoR); //Initializes 2 servos for claw
        testHorizontalLinSlide.initHori(HL); //Initializes DcMotor for horizontal linear slide
        topMotor.initTopMotor(TopMotor);
        TwoStageLinSlideFileNew.setLSMotor(rightLinSlide, leftLinSlide); //defines motors in terms of the seperate file

        //set zero power behavior to brake
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLinSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLinSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        HL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        TopMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        HL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        HL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);



        waitForStart();
        Boolean ModesTrans = false;
        int Modes = 1;
        int ModesTransTo = 1;
        int slideModes = 1;
        if (isStopRequested()) return;
        while (opModeIsActive()){

            if(gamepad2.a){
                ModesTrans = true;
                ModesTransTo = 1;
            }
            if(gamepad2.b){
                ModesTrans = true;
                ModesTransTo = 2;
            }
            if(ModesTrans){
                //code at home: statemachine for servos
                while (rightLinSlide.getCurrentPosition()!= 0 && TopMotor.getCurrentPosition() != 0){
                    if(rightLinSlide.getCurrentPosition() > 0){
                        rightLinSlide.setTargetPosition(0);
                        rightLinSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        leftLinSlide.setTargetPosition(0);
                        leftLinSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightLinSlide.setPower(-1);
                        leftLinSlide.setPower(-1);
                    }
                    else{
                        rightLinSlide.setPower(0);
                        leftLinSlide.setPower(0);
                    }
                    if(TopMotor.getCurrentPosition() > 0) {
                        TopMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        TopMotor.setTargetPosition(0);
                        TopMotor.setPower(0.9);
                    }
                    else{
                        TopMotor.setPower(0);
                    }
                    ServoTele.open(true);
                    testHorizontalLinSlide.moveHorizontalLinManual(true, false);
                }

                if(ModesTransTo == 1){
                    Modes = 1;
                }
                else if (ModesTransTo ==2){
                    Modes =2;
                    farmMode.initFarm();
                }
                ModesTrans = false;
            }
            if(gamepad2.x && rightLinSlide.getCurrentPosition() <= 0){
                slideModes = 1;
            }
            if(gamepad2.y && rightLinSlide.getCurrentPosition() <=0){
                slideModes = 2;
            }
            if (Modes == 1){
                topMotor.moveTopMotor(gamepad1.dpad_right, gamepad1.dpad_left);
                clawServoClass.spinClawServo(gamepad1.dpad_up,gamepad1.dpad_down);
                testHorizontalLinSlide.moveHorizontalLinManual(gamepad1.a,gamepad1.b);
                ServoTele.open(gamepad1.x);
                ServoTele.close(gamepad1.y,350);
                //pieceTogether.pieceTogether(gamepad1);
                telemetry.addData("Mode = ", "Manual");
                telemetry.update();
                if(slideModes == 1){
                    telemetry.addData("Slide Mode = ", "Switch State");
                    telemetry.update();
                    TwoStageLinSlideFileNew.linSlideDouble(gamepad1); //takes gamepad input
                }
                else if (slideModes == 2){
                    telemetry.addData("Slide Mode = ", "Manual");
                    telemetry.update();
                    if (gamepad1.right_trigger > 0.3&&rightLinSlide.getCurrentPosition() <=4000){
                        rightLinSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        leftLinSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        rightLinSlide.setPower(1);
                        leftLinSlide.setPower(1);
                    }
                    else if(gamepad1.left_trigger>0.3 && rightLinSlide.getCurrentPosition() >=0){
                        rightLinSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        leftLinSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        rightLinSlide.setPower(-1);
                        leftLinSlide.setPower(-1);
                    }
                    else if(gamepad1.right_bumper &&rightLinSlide.getCurrentPosition() <=4000){
                        rightLinSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        leftLinSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        rightLinSlide.setPower(0.4);
                        leftLinSlide.setPower(0.4);
                    }
                    else if(gamepad1.left_bumper&&rightLinSlide.getCurrentPosition() >=0){
                        rightLinSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        leftLinSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        rightLinSlide.setPower(-0.4);
                        leftLinSlide.setPower(-0.4);
                    } else {
                        rightLinSlide.setPower(0);
                        leftLinSlide.setPower(0);
                    }

                }

                telemetry.addData("Position", rightLinSlide.getCurrentPosition());
                telemetry.addData("ServoPositionR", ClawServoR.getPosition());
                telemetry.addData("ServoPositionL", ClawServoL.getPosition());
                telemetry.addData("PositionH", HL.getCurrentPosition());
                telemetry.update();
                double speedPosition = Math.abs(gamepad2.left_stick_y);
                double y = -gamepad1.left_stick_y; // Remember, this is reversed!
                double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
                double rx = gamepad1.right_stick_x ;
                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio, but only when
                // at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double frontLeftPower = (y + x + rx)/ denominator;
                double backLeftPower = (y - x + rx)/ denominator;
                double frontRightPower = (y - x - rx)/ denominator;
                double backRightPower = (y + x - rx)/ denominator;
                if(speedPosition == 0){
                    motorFrontLeft.setPower(frontLeftPower*0.65);
                    motorBackLeft.setPower(backLeftPower*0.65);
                    motorFrontRight.setPower(frontRightPower*0.65);
                    motorBackRight.setPower(backRightPower*0.65);
                }
                else{
                    motorFrontLeft.setPower(frontLeftPower*speedPosition);
                    motorBackLeft.setPower(backLeftPower*speedPosition);
                    motorFrontRight.setPower(frontRightPower*speedPosition);
                    motorBackRight.setPower(backRightPower*speedPosition);
                }
            }
            if(Modes == 2){
                farmMode.farm();
               /* pieceTogether.load();
                Thread.sleep(200);
                pieceTogether.upDrop();
                Thread.sleep(200);
                pieceTogether.armDown();
                Thread.sleep(200); */
            }




            /*TwoStageLinSlideFileNew.linSlideDouble(gamepad1); //takes gamepad input
            topServo.manualTSControl(gamepad1.dpad_left, gamepad1.dpad_right);
            clawServoClass.spinClawServo(gamepad1.dpad_up,gamepad1.dpad_down);
            pieceTogether.pieceTogether(gamepad1);
            telemetry.addData("Position", rightLinSlide.getCurrentPosition());
            telemetry.addData("ServoPositionR", ClawServoR.getPosition());
            telemetry.addData("ServoPositionL", ClawServoL.getPosition());
            telemetry.update();
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x ;
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) * 0.65/ denominator;
            double backLeftPower = (y - x + rx) * 0.65/ denominator;
            double frontRightPower = (y - x - rx) * 0.65/ denominator;
            double backRightPower = (y + x - rx)* 0.65/ denominator;
            motorFrontLeft.setPower(-frontLeftPower*0.6);
            motorBackLeft.setPower(-backLeftPower*0.6);
            motorFrontRight.setPower(-frontRightPower*0.6);
            motorBackRight.setPower(-backRightPower*0.6);*/

        }
    }

}


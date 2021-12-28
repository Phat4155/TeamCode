package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class Hardware4 {

        boolean y = false;
        public DcMotor rightFront = null;
        public DcMotor leftFront = null;
        public DcMotor rightBack = null;
        public DcMotor leftBack = null;
        public DcMotor Gecko = null;
        public DcMotor Lift = null;
        public DcMotor Intake = null;

        public Servo Roll = null;
        private ElapsedTime period  = new ElapsedTime();
        
        HardwareMap HardwareMap4 = null;

        private boolean isRunning = true;

        public ElapsedTime runTime = new ElapsedTime();
        public boolean buttonPressed = false;
        public double lastTick = 0;
        public double lastTime = 0;
        public double currentTick = 0;
        public double currentTime = 0;
        public double currentShooterPower = 0;
        public double targetShooterRPM = 0;
        /**
         * Method: controlShooter()
         *  -   controls the speed of the motors while the program is running
         */
        public void controlShooter() {
            this.currentShooterPower = shooterPower();
            setShooterPower(this.currentShooterPower);
        }   //end of controlShooter()

        /**
         * Method: setShooterPower(double powerLevel)
         *  -   apply the shooter power to the shooter motor(s)
         */
        public void setShooterPower(double powerLevel) {
            Gecko = HardwareMap4.get(DcMotor.class, "Gek");
            Gecko.setDirection(DcMotorSimple.Direction.REVERSE);
            Gecko.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.Gecko.setPower(powerLevel);
        }   //end of setShooterPower(double powerLevel)

        /**
         * Method: setRPMMeasurements()
         *  -   sets the values for lastTick, currentTick, lastTime, currentTime
         */
        public void setRPMMeasurments() {
            Gecko = HardwareMap4.get(DcMotor.class, "Gek");
            Gecko.setDirection(DcMotorSimple.Direction.REVERSE);
            Gecko.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            double tempValue = 0;
            this.lastTime = this.currentTime;
            this.lastTick = this.currentTick;

            for(int i = 0; i < 2000000; i++) {
                tempValue = tempValue + i;
            }   // end of for(int i = 0; i < 2000000; i++)

            this.currentTick = Gecko.getCurrentPosition();
            this.currentTime = runTime.time();
        }   //end of setRPMMeasurments()

        /**
         *
         */
        public void setTargetShooterRPM(double targetRPM) {
            this.targetShooterRPM = targetRPM;
        }   //end of setTargetShooterRPM(double targetRPM)

        /**
         * Method: measureRPM()
         *  -   measures the current RPM of the motors
         */
        public double  measureRPM() {
            setRPMMeasurments();
            double rPM = (((Math.abs(this.currentTick - this.lastTick))/537.6)/(Math.abs(this.currentTime - this.lastTime)))*60;

            return rPM;
        }   //end of measureRPM()

        /**
         * Method: stopShooterThread
         * -    stops the shooter thread
         */
        public void stopShooterThread() {
            this.isRunning = false;
        }

        /**
         * Method: shooterPower()
         * -    calculates the power for the Gecko
         */
        public double shooterPower() {
            double shooterPower = this.currentShooterPower;
            double error = this.targetShooterRPM - measureRPM();

            double Cp = 1; //aka kP
            double Ci = 0; //aka kI
            double Cd = .3; //aka kD

            double maxPower = 1;

            double integral = 0;
            double derivative = 0;

            if(this.targetShooterRPM == 0) {
                shooterPower = 0;
            } else if (this.targetShooterRPM != 0 && shooterPower == 0) {
                shooterPower = 0.50;
            } else {
                shooterPower = shooterPower + ((Cp*error) + (Ci*integral) + (Cd*derivative));
            }   //end of if(this.targetShooterRPM == 0)

            return (Range.clip(shooterPower, 0, maxPower));
        }   //end of shooterPower()
        
        public void init(HardwareMap hmap){
        
        HardwareMap4 = hmap;

        leftFront = hmap.get(DcMotor.class, "lf");
        rightFront = hmap.get(DcMotor.class, "rf");
        leftBack = hmap.get(DcMotor.class, "lb");
        rightBack = hmap.get(DcMotor.class, "rb");
        Gecko = hmap.get(DcMotor.class, "Gek");
        //LaunchLeft = hmap.get(DcMotor.class, "LL");
        //LaunchRight = hmap.get(DcMotor.class, "LR");
        Lift = hmap.get(DcMotor.class, "L");
        Intake = hmap.get(DcMotor.class, "I");

        Roll = hmap.get(Servo.class, "R");
        
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        Roll.setPosition(1.5);
        
        
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        Lift.setPower(0);
        
        }

        public void Forward(double seconds){
            period.reset();
            
            while (period.milliseconds() < (seconds * 1000)){
                leftFront.setPower(0.9);
                rightFront.setPower(1);
                leftBack.setPower(1);
                rightBack.setPower(1);
                
            }    
        }
        
        public void Backward(double seconds){
            period.reset();
            
            while (period.milliseconds() < (seconds * 1000)){
                leftFront.setPower(-0.9);
                rightFront.setPower(-1);
                leftBack.setPower(-1);
                rightBack.setPower(-1);
                
            }    
        }
        public void Left(double seconds){
            period.reset();
            
            while (period.milliseconds() < (seconds * 1000)){
                leftFront.setPower(-1);
                rightFront.setPower(1);
                leftBack.setPower(-1);
                rightBack.setPower(1);
                
            }    
        }
        public void Right(double seconds){
            period.reset();
            
            while (period.milliseconds() < (seconds * 1000)){
                leftFront.setPower(1);
                rightFront.setPower(-1);
                leftBack.setPower(1);
                rightBack.setPower(-1);
                
            }    
        }
        public void RightStrafe(double seconds){
            period.reset();
            
            while (period.milliseconds() < (seconds * 1000)){
                leftFront.setPower(1);
                rightFront.setPower(-1);
                leftBack.setPower(-1);
                rightBack.setPower(1);
                
            }    
        }
        public void RightStrafeForever(){
            leftFront.setPower(1);
            rightFront.setPower(-1);
            leftBack.setPower(-1);
            rightBack.setPower(1);
        }
        public void LeftStrafe(double seconds){
            period.reset();
            
            while (period.milliseconds() < (seconds * 1000)){
                leftFront.setPower(-1);
                rightFront.setPower(1);
                leftBack.setPower(1);
                rightBack.setPower(-1);
                
            }    
        }
        public void Off(){

            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
            Lift.setPower(0);
                
        }
        public void Should(double seconds){
              
               double time = (seconds * 1000) + period.milliseconds();
               while (time > period.milliseconds()){
                  y=y;
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
               }
               
    }
        public void Down(double seconds){
              
               double time = (seconds * 1000) + period.milliseconds();
               while (time > period.milliseconds()){
                  //y=y;
                Lift.setPower(-1);
               }
               
    }
        public void Up(double seconds){
              
               double time = (seconds * 1000) + period.milliseconds();
               while (time > period.milliseconds()){
                  //y=y;
                Lift.setPower(1);
               }
    }
        public void Grab(double position){
                Roll.setPosition(position);
    }

        /*public void Shoot(double seconds){
            
                double time = (seconds * 1000) + period.milliseconds();
                while (time > period.milliseconds()){
                    //y=y;
                 LaunchLeft.setPower(.34);
                 LaunchRight.setPower(.34);
                }
    }*/
        public void Roll(double position){
                 Roll.setPosition(position);
                }
    }

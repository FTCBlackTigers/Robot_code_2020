package org.firstinspires.ftc.teamcode.utils;

public class GlobalVariables {
     private static double endAutoRobotAngle;
     public static int endAutoLiftVertical;
     public static int endAutoLiftHorizontal;

     private GlobalVariables(){}

     public static void reset (){
         endAutoRobotAngle = 0;
         endAutoLiftVertical = 0;
         endAutoLiftHorizontal = 0;
     }

    public static void setEndAutoRobotAngle(double endAutoRobotAngle) {
        while (endAutoRobotAngle > 180) endAutoRobotAngle -= 360;
        while (endAutoRobotAngle <= -179)  endAutoRobotAngle += 360;

        GlobalVariables.endAutoRobotAngle = endAutoRobotAngle;
    }

    public static double getEndAutoRobotAngle() {
        return endAutoRobotAngle;
    }
}

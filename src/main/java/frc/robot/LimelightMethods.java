/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


/**
 * Add your docs here.
 */
public class LimelightMethods {

    // Default Method
    public static double[] autoAlign(double[] camtranData){
        double sideValue = 0.0, rotateValue = 0.0, forwardValue = 0.0;
        double KpR = 0.015;
        double KpS = 0.01;
        
        if((camtranData[4] >= 0.2 || camtranData[4] <= -0.2) && camtranData[4] != 0.0){
          rotateValue = camtranData[4] * KpR;
        }
        if((camtranData[0] >= 2 || camtranData[0] <= -2) && camtranData[0] != 0.0){
           sideValue = camtranData[0] * KpS;
        } else {
            forwardValue = 0.2;
        }
      
        return new double[]{sideValue, forwardValue, rotateValue};
    }
}


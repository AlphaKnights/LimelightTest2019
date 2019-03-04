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
    public static double[] AutoAlign(double x, double y, double forwardValue){
        double sideValue, rotateValue;
        
        if (x < -2){
            sideValue = 0.15;
          }
          else if (x > 2){
            sideValue = -0.15;
          }
          else {
            sideValue = 0;
          }
      
          if (x < -2){
            rotateValue = -0.1;
          }
          else if (x > 2){
            rotateValue = 0.1;
          }
          else {
            rotateValue = 0;
          }
      
        return new double[]{sideValue, forwardValue, rotateValue};
    }

    // Override for no forward value. Why is it done this way!?!?
    public static double[] AutoAlign(double x, double y){
        double sideValue, forwardValue, rotateValue;
        
        if (x < -2){
            sideValue = 0.15;
          }
          else if (x > 2){
            sideValue = -0.15;
          }
          else {
            sideValue = 0;
          }
      
          if (x < -2){
            rotateValue = -0.2;
          }
          else if (x > 2){
            rotateValue = 0.2;
          }
          else {
            rotateValue = 0;
          }

          if (rotateValue == 0 && sideValue == 0){
            forwardValue = 0.15;
          } else {
            forwardValue = 0;
          }
      
        return new double[]{sideValue, forwardValue, rotateValue};
    }

    //for non-rotational movement
    /* public static double[] AutoAlign2(double x, double y){
      double sideValue, forwardValue, rotateValue;
      
      if (x < -2){
          sideValue = 0.15;
        }
        else if (x > 2){
          sideValue = -0.15;
        }
        else {
          sideValue = 0;
        }
    
       
    
      return new double[]{sideValue, forwardValue, rotateValue};
  } */
}


package us.ihmc.steppr.hardware.state;

import cern.colt.Arrays;

public class StepprAnkleInterpolator implements StepprAnkleAngleCalculator
{
   //      private static final double px[] = { 0.000000000009934, -1.269023940640927, 1.269023940648429, -0.020516021439826, 0.020516020934548, 0.000000000508670,
   //            -0.115495045972189, 0.523312234721911, -0.523312234476478, 0.115495045733103 }; //parameters for cubic mapping of pulley angles to ankle x
   //   
   //      private static final double py[] = { 0.000022001170581, 0.463732921796384, 0.463732921942751, -0.048394600966524, -0.048394600971605, 0.111755077411411,
   //            -0.075022107557441, 0.057936364110976, 0.057936369906301, -0.075022109478369 }; //parameters for cubic mapping of pulley angles to ankle x
   //   
   //      private static final double pJitX[] = { -1.272997428093503, -0.036001688534274, -0.001270575344824, -0.411410632841345, -0.628599882888178,
   //            1.233612286706826, 0.128100494751386, -0.209457296600427, -0.014034155341702, 0.080071535526900 }; //parameters for cubic mapping of pulley angles to jacobian inverse transpose element 1,1
   //   
   //      private static final double pJitY[] = { 0.465442714729626, -0.079404497795577, 0.091522713969963, -0.239129828847536, 0.040327663912797, 0.141924039046489,
   //            -0.130148705434543, 0.342409812822250, -0.315199100250677, 0.111764660552245 }; //parameters for cubic mapping of pulley angles to jacobian inverse transpose element 2,1

   private static final double px[] = { 0.000000000011833, 1.269023941053808, -1.269023941048548, 0.020516020869627, -0.020516021471959, 0.000000000606667,
         0.115495040111334, -0.523312217421704, 0.523312217621650, -0.115495040305941 };

   private static final double py[] = { 0.000022001174462, 0.463732921959109, 0.463732921783293, -0.048394601071517, -0.048394601065684, 0.111755077611979,
         -0.075022109801143, 0.057936370829754, 0.057936363252024, -0.075022107299457 };

   private static final double pJitX[] = { 1.272997427777619, 0.036001701491556, 0.001270562468774, 0.411410640321290, 0.628599891388498, -1.233612302811653,
         -0.128100680179965, 0.209457856510320, 0.014033597275913, -0.080071351885635 };

   private static final double pJitY[] = { 0.465442714987602, -0.079404498198185, 0.091522714637596, -0.239129834635615, 0.040327656123728, 0.141924052147264,
         -0.130148712431276, 0.342409832258836, -0.315199115178924, 0.111764662233772 };

   private static final int N = 6; // Cable reduction of pulleys

   // this returns the value a 2D cubic polynomial with given parameters p
   // the m1,m2 inputs are the motor1 and 2 pulley angles
   private static double CubicApprox(double p[], double m1, double m2)
   {

      m1 /= N; //parameters were given for pulley angle not motor angle, corrected here
      m2 /= N; //parameters were given for pulley angle not motor angle, corrected here

      double val = p[0] + p[1] * m1 + p[2] * m2 + p[3] * m1 * m1 + p[4] * m2 * m2 + p[5] * m1 * m2 + p[6] * m1 * m1 * m1 + p[7] * m1 * m1 * m2 + p[8] * m1 * m2
            * m2 + p[9] * m2 * m2 * m2;

      return val;

   }

   //This should return a 1D array version of the Jacobian inverse transpose matrix for the linkage
   //pJX and pJY are the parameters for the cubic mappings
   //the m1,m2 inputs are the motor1 and 2 pulley angles
   private static void JacobianInverseTranspose(double[] Jit, double m1, double m2)
   {

      Jit[0] = CubicApprox(pJitX, m1, m2); //Jit11
      Jit[1] = CubicApprox(pJitY, m1, m2); //Jit12
      Jit[2] = -CubicApprox(pJitX, m2, m1); //Jit21
      Jit[3] = CubicApprox(pJitY, m2, m1); //Jit22

   }

   //with the previously defined Jacobian inverse transpose and desired ankle torques, determine the motor torques
   private void sendAnkleCommand(double m1, double m2, double X_torque_des, double Y_torque_des)
   {
      double[] Jit = new double[4];
      JacobianInverseTranspose(Jit, m1, m2);

      double m1Command = (Jit[0] * X_torque_des + Jit[1] * Y_torque_des) / N; //this is desired torque at motor 1
      double m2Command = (Jit[2] * X_torque_des + Jit[3] * Y_torque_des) / N; //this is desired torque at motor 2

      //todo send commands

   }

   public static void twobytwoInverseTranspose(double JToPack[], double Jit[])
   {

      double det = Jit[0] * Jit[3] - Jit[1] * Jit[2];
      JToPack[0] = Jit[3] / det;
      JToPack[1] = -Jit[2] / det;
      JToPack[2] = -Jit[1] / det;
      JToPack[3] = Jit[0] / det;
   }

   private final double[] Jit = new double[4];
   private double qAnkleX, qAnkleY;
   private double qdAnkleX, qdAnkleY;

   @Override
   public void updateAnkleState(double motorAngleRight, double motorAngleLeft, double motorVelocityRight, double motorVelocityLeft)
   {
      qAnkleX = CubicApprox(px, motorAngleRight, motorAngleLeft);
      qAnkleY = CubicApprox(py, motorAngleRight, motorAngleLeft);

      JacobianInverseTranspose(Jit, motorAngleRight, motorAngleLeft);

      qdAnkleX = (Jit[0] * motorVelocityRight + Jit[2] * motorVelocityLeft) / N;
      qdAnkleY = (Jit[1] * motorVelocityRight + Jit[3] * motorVelocityLeft) / N;

   }

   @Override
   public double getQAnkleX()
   {
      return qAnkleX;
   }

   @Override
   public double getQAnkleY()
   {
      return qAnkleY;
   }

   @Override
   public double getQdAnkleX()
   {
      return qdAnkleX;
   }

   @Override
   public double getQdAnkleY()
   {
      return qdAnkleY;
   }

}

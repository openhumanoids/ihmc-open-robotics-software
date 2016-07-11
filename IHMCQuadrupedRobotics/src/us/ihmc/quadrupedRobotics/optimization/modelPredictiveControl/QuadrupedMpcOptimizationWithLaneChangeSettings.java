package us.ihmc.quadrupedRobotics.optimization.modelPredictiveControl;

public class QuadrupedMpcOptimizationWithLaneChangeSettings
{
   /** maximum preview window duration in seconds */
   private double maximumPreviewTime;

   /** angular momentum cost */
   private double angularMomentumCost;

   /** step adjustment cost */
   private double stepAdjustmentCost;

   /** center of pressure adjustment cost */
   private double copAdjustmentCost;

   public QuadrupedMpcOptimizationWithLaneChangeSettings()
   {
      this(10, 1000000, 10000, 1);
   }

   public QuadrupedMpcOptimizationWithLaneChangeSettings(double maximumPreviewTime, double angularMomentumCost, double stepAdjustmentCost, double copAdjustmentCost)
   {
      this.maximumPreviewTime = maximumPreviewTime;
      this.angularMomentumCost = angularMomentumCost;
      this.stepAdjustmentCost = stepAdjustmentCost;
      this.copAdjustmentCost = copAdjustmentCost;
   }

   public double getAngularMomentumCost()
   {
      return angularMomentumCost;
   }

   public void setAngularMomentumCost(double angularMomentumCost)
   {
      this.angularMomentumCost = angularMomentumCost;
   }

   public double getStepAdjustmentCost()
   {
      return stepAdjustmentCost;
   }

   public void setStepAdjustmentCost(double stepAdjustmentCost)
   {
      this.stepAdjustmentCost = stepAdjustmentCost;
   }

   public double getCopAdjustmentCost()
   {
      return copAdjustmentCost;
   }

   public void setCopAdjustmentCost(double copAdjustmentCost)
   {
      this.copAdjustmentCost = copAdjustmentCost;
   }

   public double getMaximumPreviewTime()
   {
      return maximumPreviewTime;
   }

   public void setMaximumPreviewTime(double maximumPreviewTime)
   {
      this.maximumPreviewTime = maximumPreviewTime;
   }
}

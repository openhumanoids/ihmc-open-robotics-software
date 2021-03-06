package us.ihmc.robotics.functionApproximation;

import org.junit.Test;

import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

import java.util.Random;

import static org.junit.Assert.assertArrayEquals;

public class OnlineLinearRegressionTest {

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
	   public void toyExample()
	   {
		   int testSize=100;
		   int inputDim=10;
		   Random rnd = new Random(0);
		   
		   double[] trueCoefficient = new double[inputDim];
		   for(int i=0;i<inputDim;i++)
			   trueCoefficient[i] = rnd.nextDouble();
			   
		   
		   OnlineLinearRegression solver = new OnlineLinearRegression(100, inputDim);
		   for(int i=0;i<testSize;i++)
		   {
			   double y=0;
			   double[] x=new double[inputDim];
			   for(int j=0;j<inputDim;j++)
			   {
				   x[j]=rnd.nextDouble();
				   y+=trueCoefficient[j]*x[j];
			   }
			   solver.addEntry(y,x);
		   }
		   
		   double [] resultCoefficient = solver.getCoefficient();
		   assertArrayEquals(trueCoefficient, resultCoefficient, 1e-10);
	   }

}

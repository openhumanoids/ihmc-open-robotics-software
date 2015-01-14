package us.ihmc.atlas;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

import us.ihmc.utilities.test.JUnitTestSuiteConstructor;

@RunWith(Suite.class)
@Suite.SuiteClasses
({
   us.ihmc.atlas.networkProcessor.depthData.AtlasDepthDataProcessorTest.class,
})

public class AtlasLidarBambooJMETestSuite
{
   public static void main(String[] args)
   {
      JUnitTestSuiteConstructor.generateTestSuite(AtlasLidarBambooJMETestSuite.class);
   }
}

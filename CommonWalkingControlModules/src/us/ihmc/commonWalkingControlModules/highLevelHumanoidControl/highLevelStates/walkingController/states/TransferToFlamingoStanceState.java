package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states;

import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.commonWalkingControlModules.desiredFootStep.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotSide.RobotSide;

public class TransferToFlamingoStanceState extends TransferState
{
   public TransferToFlamingoStanceState(RobotSide transferToSide, WalkingMessageHandler walkingMessageHandler,
         HighLevelHumanoidControllerToolbox momentumBasedController, HighLevelControlManagerFactory managerFactory,
         WalkingFailureDetectionControlModule failureDetectionControlModule, YoVariableRegistry parentRegistry)
   {
      super(transferToSide, WalkingStateEnum.getFlamingoTransferState(transferToSide), walkingMessageHandler, momentumBasedController, managerFactory,
            failureDetectionControlModule, parentRegistry);
   }

   @Override
   public void doTransitionIntoAction()
   {
      super.doTransitionIntoAction();

      if (!comHeightManager.hasBeenInitializedWithNextStep())
      {
         TransferToAndNextFootstepsData transferToAndNextFootstepsDataForDoubleSupport = walkingMessageHandler.createTransferToAndNextFootstepDataForDoubleSupport(transferToSide);
         double extraToeOffHeight = 0.0;
         if (feetManager.willDoToeOff(null, transferToSide))
            extraToeOffHeight = feetManager.getWalkOnTheEdgesManager().getExtraCoMMaxHeightWithToes();
         comHeightManager.initialize(transferToAndNextFootstepsDataForDoubleSupport, extraToeOffHeight);
      }

      // Transferring to execute a foot pose, hold current desired in upcoming support foot in case it slips
      pelvisOrientationManager.setToHoldCurrentDesiredInSupportFoot(transferToSide);

      balanceManager.setSingleSupportTime(Double.POSITIVE_INFINITY);
      balanceManager.addFootstepToPlan(walkingMessageHandler.getFootstepAtCurrentLocation(transferToSide.getOppositeSide()));
      balanceManager.setICPPlanTransferToSide(transferToSide);
      balanceManager.initializeICPPlanForTransfer();
   }
}
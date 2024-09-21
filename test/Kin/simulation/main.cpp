#include <Kin/kin.h>
#include <Kin/frame.h>
#include <Kin/feature.h>
#include <Kin/simulation.h>
#include <Kin/viewer.h>
#include <Kin/F_geometrics.h>
#include <Kin/kin_physx.h>

#include <Geo/depth2PointCloud.h>

#include <iomanip>

//===========================================================================

void testPushes(){
  rai::Configuration C;
  //C.addFile(rai::raiPath("../rai-robotModels/scenarios/liftRing.g"));
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/sim_test.g"));
  C["box"]->set_Q()->setText("<t(.3 -.1 .25) d(40 1 1 0)>");
  //C["stick"]->set_Q()->setText("<t(-.3 .6 1.1) d(90 1 0 0) d(20 1 1 0)>");
  // C.view(true); // Comment out this line

  rai::Simulation S(C, S._physx, 2);

  double tau=.01;
  Metronome tic(tau);
  byteA rgb;
  floatA depth;

  arr Xstart = C.getFrameState();

  for(uint k=0;k<2;k++){

    //restart from the same state multiple times
    //S.setState(Xstart);

    for(uint t=0;t<800;t++){
      tic.waitForTic();
      //if(!(t%10)) S.getImageAndDepth(rgb, depth); //we don't need images with 100Hz, rendering is slow

      //some good old fashioned IK
      arr q = C.getJointState();
      arr diff = C.feature(FS_positionDiff, {"gripper", "box"})->eval(C);
      diff *= .02/length(diff);
      q -= pseudoInverse(diff.J(), NoArr, 1e-2) * diff;
//      C.setJointState(q);

      S.step(q, tau, S._position);
      //S.step(q, tau, S._velocity);
      //S.step(q, tau, S._acceleration);
      //S.step(q, tau, S._posVel);
      //S.step(q, tau, S._spline);
      //S.step(q, tau, S._none);

      //a crazy disturbance, lifting the box suddenly
      // if(!(t%100)){
      //   arr p = C["box"]->getPosition();
      //   p(0) += .05;
      //   p(2) += .2;
      //   C["box"]->setPosition(p);

      //   //S.setState(C.getFrameState());
      // }
    }
  }
}

//===========================================================================


//===========================================================================

int MAIN(int argc,char **argv){
  rai::initCmdLine(argc, argv);

  testPushes();

  return 0;
}

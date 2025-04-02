#include "vex.h"
#include <math.h>
#include <algorithm>
#include <fstream>

int AIVision1_objectIndex = 0;
void AIvision(){
while (true) {
    AIVision1.takeSnapshot(aivision::ALL_AIOBJS);
    if (AIVision1.objectCount > 0) {
      if (AIVision1.objects[AIVision1_objectIndex].id == mobileGoal) {

      }
      else {
        if (AIVision1.objects[AIVision1_objectIndex].id == redRing) {
          
        }
        else {
          if (AIVision1.objects[AIVision1_objectIndex].id == mobileGoal) {

          }
        }
      }
    }
  wait(5, msec);
  }
  return 0;
}
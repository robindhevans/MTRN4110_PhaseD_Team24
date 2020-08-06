// File:          PhaseD_Master.cpp
// Date:
// Description:
// Author:
// Modifications:

#include <webots/Robot.hpp>
#include <z5197018_MTRN4110_PhaseB.cpp>
#include <z5197018_MTRN4110_PhaseA.cpp>

using namespace webots;
using namespace std;

int z5197018_MTRN4110_PhaseA();
int z5197018_MTRN4110_PhaseB();

int main(int argc, char **argv) {

  while(z5197018_MTRN4110_PhaseB()) {}
  while(z5197018_MTRN4110_PhaseA()) {}
  return 0;
}
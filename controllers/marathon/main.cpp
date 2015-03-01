// File:          main.cpp
// Date:          1st of June 2011
// Description:   Manage the entree point function
// Author:        fabien.rohrer@cyberbotics.com

#include "Marathon.hpp"

#include <cstdlib>

using namespace webots;

int main(int argc, char **argv)
{
  Marathon *controller = new Marathon();
  controller->run();
  delete controller;
  return EXIT_FAILURE;
}


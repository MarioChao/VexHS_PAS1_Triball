#ifndef __GuiClass__
#define __GuiClass__

#include "vex.h"
#include <cstdio>
#include <iostream>
#include <vector>

using std::string, std::vector, std::pair, std::make_pair, std::copy;

class GuiClass {
  public:
    virtual void draw();
    void setVisibility(bool visibility);
    bool isVisible();
  private:
    bool visible = true;
};

#endif
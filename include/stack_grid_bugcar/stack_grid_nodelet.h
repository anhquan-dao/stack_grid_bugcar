// -*- lsst-c++ -*-
#pragma once

#include <stack_grid_bugcar/stack_grid_node.h>

#include <nodelet/nodelet.h>

namespace
{
    class StackGridNodelet : public StackGridBase, public nodelet::Nodelet
    {
    public:
        StackGridNodelet(){};
        ~StackGridNodelet(){};
        virtual void onInit()
        {
        }

        virtual void initialize();
    };
}
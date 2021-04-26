#pragma once
#include "krabilib/strategie/strategiev3.h"

enum GrabberContent
{
    NOTHING,
    GREEN_CUP,
    RED_CUP,
    UNKNOWN,
    ANY
};

class Grabber
{
public:
    Grabber(){};
    Grabber(Position a_relative_position);

    void release(GrabberContent type_to_release);
    void grab(GrabberContent type_to_catch);
    float getReach();

private:
    Position relative_position;
    GrabberContent content;

    void release();
    void grab();
};

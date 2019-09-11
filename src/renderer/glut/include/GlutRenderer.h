#pragma once

#include "Scene.h"
#include "GlutCamera.h"
#include <functional>

namespace glutRenderer
{

class GlutRenderer
{
typedef std::function<void(unsigned char)> GlutKeyboardHandler;
public:
    void AttachScene(::Scene* scene, GlutKeyboardHandler handler = nullptr);
    void StartRenderLoop();
    static GlutRenderer* GetInstance();
private:
    static GlutRenderer globalGlutRenderer;
private:
    ::Scene* scene;
    GlutCamera* glutCamera;
    GlutKeyboardHandler keyboardHandler;
    bool simulating;
public:
    int renderFrequenceHz;
private:
    GlutRenderer();
    void renderScene();
    void motionCallback(int x, int y);
    void keyboardCallback(unsigned char key, int x, int y);
    void mouseCallback(int button, int state, int x, int y);
    void idleCallback();
    void renderCallback();
    void exitCallback();
};

}
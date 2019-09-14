#pragma once

#include "Scene.h"
#include "GlutCamera.h"
#include <functional>

namespace glutRenderer
{

class GlutRendererCallback
{
// API BEGIN
public:
    virtual void keyboardHandler(unsigned char key) = 0;
    virtual void beforeSimulationHandler() = 0;
    virtual ~GlutRendererCallback() {}
// API END
};

class GlutRenderer
{
// API BEGIN
public:
    void AttachScene(::Scene* scene, GlutRendererCallback* callback = nullptr);
    void StartRenderLoop();
    static GlutRenderer* GetInstance();
    GlutRenderer(); // For API only
public:
    int renderFrequenceHz;
// API END
private:
    static GlutRenderer globalGlutRenderer;
private:
    ::Scene* scene;
    GlutCamera* glutCamera;
    GlutRendererCallback* callback;
    bool simulating;
private:
    void renderScene();
    void motionCallback(int x, int y);
    void keyboardCallback(unsigned char key, int x, int y);
    void mouseCallback(int button, int state, int x, int y);
    void idleCallback();
    void renderCallback();
    void exitCallback();
};

}

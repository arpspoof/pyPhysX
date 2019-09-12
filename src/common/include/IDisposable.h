#pragma once

// Interface only
class IDisposable
{
public:
    virtual void Dispose() = 0;
    virtual ~IDisposable() {}
};

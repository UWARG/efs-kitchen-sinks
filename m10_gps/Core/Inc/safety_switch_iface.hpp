#pragma once

// Interface used by higher level logic to query the current safety switch state.
class ISafetySwitch
{
protected:
    ISafetySwitch() = default;

public:
    virtual ~ISafetySwitch() = default;
    virtual bool isPressed() const = 0;
    virtual void buzzerOn() const = 0;
    virtual void buzzerOff() const = 0;

};

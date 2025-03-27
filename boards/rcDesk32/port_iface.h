#pragma once

struct PortDevice {
    virtual void periodic() = 0;
    virtual void ratePeriodic() = 0;
    virtual ~PortDevice() {}
};
struct IAux : PortDevice {
    // virtual void update() = 0;
};
template<typename A>
struct Aux: IAux {
    Aux() {
        A::init();
    }
    ~Aux() {
        A::reset();
    }
    // virtual void update() {
    //     A::update();
    // }
    virtual void periodic() {
        A::periodic();
    }
    virtual void ratePeriodic() {
        A::ratePeriodic();
    }
};

struct ISm : PortDevice {
    // virtual void update() = 0;
};
template<typename S>
struct Sm: ISm{
    Sm() {
        S::init();
    }
    ~Sm() {
        S::reset();
    }
    // virtual void update() {
    //     S::update();
    // }
    virtual void periodic() {
        S::periodic();
    }
    virtual void ratePeriodic() {
        S::ratePeriodic();
    }
};


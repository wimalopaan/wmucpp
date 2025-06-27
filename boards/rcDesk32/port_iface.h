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
        if constexpr(requires(){A::periodic();}) {
            A::periodic();
        }
    }
    virtual void ratePeriodic() {
        if constexpr(requires(){A::ratePeriodic();}) {
            A::ratePeriodic();
        }
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
        if constexpr(requires(){S::periodic();}) {
            S::periodic();
        }
    }
    virtual void ratePeriodic() {
        if constexpr(requires(){S::ratePeriodic();}) {
            S::ratePeriodic();
        }
    }
};

struct IEnc : PortDevice {
    // virtual void update() = 0;
};
template<typename E>
struct Enc: IEnc {
    Enc() {
        E::init();
    }
    ~Enc() {
        E::reset();
    }
    virtual void periodic() {
        if constexpr(requires(){E::periodic();}) {
            E::periodic();
        }
    }
    virtual void ratePeriodic() {
        if constexpr(requires(){E::ratePeriodic();}) {
            E::ratePeriodic();
        }
    }
};

struct IBus : PortDevice {
    // virtual void update() = 0;
};
template<typename B>
struct Bus: IBus {
    Bus() {
        B::init();
    }
    ~Bus() {
        B::reset();
    }
    virtual void periodic() {
        if constexpr(requires(){B::periodic();}) {
            B::periodic();
        }
    }
    virtual void ratePeriodic() {
        if constexpr(requires(){B::ratePeriodic();}) {
            B::ratePeriodic();
        }
    }
};

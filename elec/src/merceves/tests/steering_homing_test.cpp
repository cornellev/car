#include "../steering_homing.hpp"
#include <cassert>

int main() {
    SteeringHoming h;
    assert(!h.active());  // Boot alone cannot move steering.
    h.start(0);
    assert(h.update(0, 3220, true, true) == 1);
    // First encounter does not qualify if we leave before 200 ms.
    assert(h.update(5, 1700, true, false) == 1);
    assert(h.update(100, 2100, true, false) == 1);
    assert(h.update(105, 2101, true, false) == 1);
    assert(h.update(300, 4090, true, false) == 1);
    assert(h.update(305, 10, true, false) == 1);
    assert(h.update(400, 1699, true, false) == 1);
    assert(h.update(405, 1700, true, false) == 1);
    // Large movements inside the window do not reset the timer.
    assert(h.update(500, 2100, true, false) == 1);
    assert(h.update(604, 1800, true, false) == 1);
    assert(h.update(605, 1750, true, false) == 0);
    assert(h.state == SteeringHoming::State::drain_right);
    assert(h.update(610, 1740, true, false) == 0); // Wait for pending pulses.
    assert(h.update(615, 1740, true, true) == -1);
    assert(h.right_bound == 1740);
    h.update(620, 1000, true, false);
    h.update(625, 10, true, false);
    h.update(630, 4090, true, false); // Return left through wrap.
    h.update(635, 3300, true, false);
    assert(h.update(640, 3237, true, false) == -1);
    assert(h.update(645, 3236, true, false) == 0); // Center +16.
    assert(h.state == SteeringHoming::State::drain_center);
    h.update(650, 3225, true, true);
    assert(h.state == SteeringHoming::State::ready);

    // A session restart must start a fresh wall timer.
    h.start(1000);
    h.update(1000, 1800, true, true);
    h.reset();
    h.start(1200);
    assert(h.update(1200, 1800, true, true) == 1);
    assert(h.update(1399, 1800, true, false) == 1);
    assert(h.update(1400, 1800, true, false) == 0);

    h.reset();
    h.start(2000);
    h.update(2000, 0, false, true);
    assert(h.state == SteeringHoming::State::failed);
    h.reset();
    h.start(3000);
    h.update(18001, 1700, true, true);
    assert(h.state == SteeringHoming::State::failed);
}

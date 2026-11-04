#include "../steering_angle.hpp"
#include <cassert>
#include <cmath>

int main() {
    SteeringAngle angle;
    assert(std::isnan(angle.degrees()));
    angle.reference(3220, 0);
    assert(angle.degrees() == 0);
    angle.update(4000, true, 5);
    angle.update(100, true, 10);
    angle.update(1700, true, 15);
    assert(std::abs(angle.degrees() + 2576 * STEERING_DEGREES_PER_ENCODER_INCREMENT) < 0.00001);
    angle.update(100, true, 20);
    angle.update(4000, true, 25);
    angle.update(3220, true, 30);
    assert(angle.degrees() == 0);
    // More than one revolution left, including wrap in the other direction.
    angle.update(1500, true, 35);
    angle.update(0, true, 40);
    angle.update(4090, true, 45);
    angle.update(3220, true, 50);
    assert(std::abs(angle.degrees() - 4096 * STEERING_DEGREES_PER_ENCODER_INCREMENT) < 0.00001);
    angle.update(0, false, 55);
    assert(std::isnan(angle.degrees()));
    angle.update(3220, true, 60);
    assert(std::isfinite(angle.degrees()));
    angle.update(3220, true, 161);
    assert(std::isnan(angle.degrees())); // Cannot guess wraps after a long gap.
    angle.reference(3230, 170);
    assert(std::abs(angle.degrees() + 10 * STEERING_DEGREES_PER_ENCODER_INCREMENT) < 0.00001);
    angle.reset();
    assert(std::isnan(angle.degrees()));
}

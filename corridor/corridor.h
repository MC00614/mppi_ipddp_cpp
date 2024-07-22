#include "collision_checker.h"

class Corridor {
public:
    Corridor();
    ~Corridor();
    void setCollisionChecker(CollisionChecker *collision_checker);
private:
    CollisionChecker *collision_checker;
};

Corridor::Corridor() {
}

Corridor::~Corridor() {
}

void Corridor::setCollisionChecker(CollisionChecker *collision_checker) {
    this->collision_checker = collision_checker;
}


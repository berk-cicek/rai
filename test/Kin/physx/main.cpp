#include <iostream>
#include "PxPhysicsAPI.h"

using namespace physx;

int main() {
    // Initialize PhysX Foundation
    PxFoundation* foundation = PxCreateFoundation(PX_PHYSICS_VERSION, PxDefaultAllocator(), PxDefaultErrorCallback());
    if (!foundation) {
        std::cerr << "Failed to initialize PhysX Foundation" << std::endl;
        return -1;
    }

    // Initialize PhysX SDK
    PxPhysics* physics = PxCreatePhysics(PX_PHYSICS_VERSION, *foundation, PxTolerancesScale());
    if (!physics) {
        std::cerr << "Failed to initialize PhysX SDK" << std::endl;
        foundation->release();
        return -1;
    }

    // Print the PhysX SDK version
    std::cout << "PhysX SDK Version: " << PX_PHYSICS_VERSION << std::endl;

    // Cleanup
    physics->release();
    foundation->release();

    return 0;
}
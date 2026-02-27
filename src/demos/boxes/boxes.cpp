/*
 * A simple two-box stacking demo.
 *
 * Part of the Cyclone physics system.
 */

#include <cyclone/cyclone.h>
#include <cstdio>
#include "../ogl_headers.h"
#include "../app.h"
#include "../timing.h"

class Box : public cyclone::CollisionBox
{
public:
    Box()
    {
        body = new cyclone::RigidBody();
    }

    ~Box()
    {
        delete body;
    }

    void setState(const cyclone::Vector3 &position,
                  const cyclone::Vector3 &halfSize,
                  cyclone::real mass)
    {
        this->halfSize = halfSize;

        body->setPosition(position);
        body->setOrientation(1, 0, 0, 0);
        body->setVelocity(0, 0, 0);
        body->setRotation(0, 0, 0);
        body->setMass(mass);

        cyclone::Matrix3 tensor;
        tensor.setBlockInertiaTensor(halfSize, mass);
        body->setInertiaTensor(tensor);

        body->setLinearDamping((cyclone::real)0.95);
        body->setAngularDamping((cyclone::real)0.8);
        body->setAcceleration(cyclone::Vector3::GRAVITY);
        body->clearAccumulators();
        body->setCanSleep(true);
        body->calculateDerivedData();

        calculateInternals();
    }

    void render() const
    {
        GLfloat mat[16];
        body->getGLTransform(mat);

        if (body->getAwake()) glColor3f(0.85f, 0.5f, 0.4f);
        else glColor3f(0.45f, 0.55f, 0.85f);

        glPushMatrix();
        glMultMatrixf(mat);
        glScalef(halfSize.x * 2, halfSize.y * 2, halfSize.z * 2);
        glutSolidCube(1.0f);
        glPopMatrix();
    }
};

class BoxesDemo : public RigidBodyApplication
{
    class BoxesCollisionLogger : public cyclone::ContactResolverDebugListener
    {
        cyclone::RigidBody *trackedBodies[2];
        FILE *output;
        unsigned frameWindowAfterCollisionStart;
        unsigned frameCounter;
        unsigned capturedFramesSinceCollisionStart;
        bool wasCollidingLastFrame;
        bool captureActive;
        bool activeFrame;

        static void printVector(FILE *output, const cyclone::Vector3 &value)
        {
            std::fprintf(
                output,
                "(%.6f, %.6f, %.6f)",
                (double)value.x, (double)value.y, (double)value.z
            );
        }

        static void printQuaternion(FILE *output, const cyclone::Quaternion &value)
        {
            std::fprintf(
                output,
                "(%.6f, %.6f, %.6f, %.6f)",
                (double)value.r, (double)value.i, (double)value.j, (double)value.k
            );
        }

        static void printMatrix(FILE *output, const cyclone::Matrix3 &value)
        {
            std::fprintf(
                output,
                "[%.6f %.6f %.6f | %.6f %.6f %.6f | %.6f %.6f %.6f]",
                (double)value.data[0], (double)value.data[1], (double)value.data[2],
                (double)value.data[3], (double)value.data[4], (double)value.data[5],
                (double)value.data[6], (double)value.data[7], (double)value.data[8]
            );
        }

        bool isTrackedBody(const cyclone::RigidBody *body) const
        {
            return body == trackedBodies[0] || body == trackedBodies[1];
        }

        bool isTrackedPair(const cyclone::Contact &contact) const
        {
            return contact.body[0] && contact.body[1] &&
                isTrackedBody(contact.body[0]) &&
                isTrackedBody(contact.body[1]) &&
                contact.body[0] != contact.body[1];
        }

        bool isSecondBoxGroundContact(const cyclone::Contact &contact) const
        {
            return trackedBodies[1] &&
                ((contact.body[0] == trackedBodies[1] && !contact.body[1]) ||
                 (contact.body[1] == trackedBodies[1] && !contact.body[0]));
        }

        void printTrackedBodyState(const char *label, const cyclone::RigidBody *body)
        {
            cyclone::Vector3 position = body->getPosition();
            cyclone::Quaternion orientation = body->getOrientation();
            cyclone::Vector3 velocity = body->getVelocity();
            cyclone::Vector3 rotation = body->getRotation();
            const cyclone::real motion = body->getMotion();

            std::fprintf(output, "  %s: ", label);
            std::fprintf(output, "position=");
            printVector(output, position);
            std::fprintf(output, ", orientation=");
            printQuaternion(output, orientation);
            std::fprintf(output, ", velocity=");
            printVector(output, velocity);
            std::fprintf(output, ", rotation=");
            printVector(output, rotation);
            std::fprintf(output, ", motion=%.6f\n", (double)motion);
        }

        void printContactSummary(const cyclone::Contact &contact)
        {
            std::fprintf(output, "point=");
            printVector(output, contact.contactPoint);
            std::fprintf(output, ", normal=");
            printVector(output, contact.contactNormal);
            std::fprintf(output, ", depth=%.6f", (double)contact.penetration);
        }

    public:
        BoxesCollisionLogger(unsigned frameWindowAfterCollisionStart)
            : output(NULL),
              frameWindowAfterCollisionStart(frameWindowAfterCollisionStart),
              frameCounter(0),
              capturedFramesSinceCollisionStart(0),
              wasCollidingLastFrame(false),
              captureActive(false),
              activeFrame(false)
        {
            trackedBodies[0] = NULL;
            trackedBodies[1] = NULL;
            output = std::fopen("boxes_collision.log", "w");
        }

        ~BoxesCollisionLogger()
        {
            if (output) std::fclose(output);
        }

        void setTrackedBodies(cyclone::RigidBody *first, cyclone::RigidBody *second)
        {
            trackedBodies[0] = first;
            trackedBodies[1] = second;
        }

        void beginFrame(cyclone::real duration, bool boxesColliding)
        {
            frameCounter++;

            if (boxesColliding && !wasCollidingLastFrame)
            {
                captureActive = true;
                capturedFramesSinceCollisionStart = 0;
            }

            activeFrame = captureActive &&
                (capturedFramesSinceCollisionStart < frameWindowAfterCollisionStart);

            if (!activeFrame || !output)
            {
                wasCollidingLastFrame = boxesColliding;
                if (captureActive &&
                    capturedFramesSinceCollisionStart >= frameWindowAfterCollisionStart)
                {
                    captureActive = false;
                }
                return;
            }

            std::fprintf(output, "============================================================\n");
            std::fprintf(
                output,
                "Frame %u duration=%.6f boxesColliding=%s\n",
                frameCounter,
                (double)duration,
                boxesColliding ? "true" : "false"
            );
            printTrackedBodyState("box[0]", trackedBodies[0]);
            printTrackedBodyState("box[1]", trackedBodies[1]);

            capturedFramesSinceCollisionStart++;
            if (capturedFramesSinceCollisionStart >= frameWindowAfterCollisionStart)
            {
                captureActive = false;
            }

            wasCollidingLastFrame = boxesColliding;
        }

        void endFrame()
        {
            if (output && activeFrame) std::fflush(output);
        }

        virtual bool shouldLogContact(const cyclone::Contact &contact) const
        {
            return activeFrame;
        }

        virtual void onPrepareContact(unsigned contactIndex,
                                      const cyclone::Contact &contact,
                                      cyclone::real duration,
                                      const cyclone::Matrix3 &contactToWorld,
                                      cyclone::real desiredDeltaVelocity)
        {
            if (!output) return;
            std::fprintf(output, "  [prepareContacts]\n");
            std::fprintf(output, "      contact=%u\n", contactIndex);
            std::fprintf(output, "      point=");
            printVector(output, contact.contactPoint);
            std::fprintf(output, "\n");
            std::fprintf(output, "      normal=");
            printVector(output, contact.contactNormal);
            std::fprintf(output, "\n");
            std::fprintf(output, "      relative_pos[0]=");
            printVector(output, contact.relativeContactPosition[0]);
            std::fprintf(output, "\n");
            std::fprintf(output, "      relative_pos[1]=");
            printVector(output, contact.relativeContactPosition[1]);
            std::fprintf(output, "\n");
            std::fprintf(output, "      depth=%.6f\n", (double)contact.penetration);
            std::fprintf(output, "      basis=");
            printMatrix(output, contactToWorld);
            std::fprintf(output, "\n");
            std::fprintf(output, "      local_velocity=");
            printVector(output, contact.contactVelocity);
            std::fprintf(output, "\n");
            std::fprintf(output, "      desiredDeltaVelocity=%.6f\n", (double)desiredDeltaVelocity);
        }

        virtual void onPositionIteration(unsigned iteration)
        {
            if (!output || !activeFrame) return;
            std::fprintf(output, "  [adjustPositions] iteration=%u\n", iteration);
        }

        virtual void onPositionResolution(unsigned iteration,
                                          unsigned contactIndex,
                                          const cyclone::Contact &contact,
                                          const cyclone::Matrix3 &contactToWorld,
                                          cyclone::real desiredDeltaVelocity,
                                          const cyclone::Vector3 linearChange[2],
                                          const cyclone::Vector3 angularChange[2])
        {
            if (!output) return;
            std::fprintf(output, "    [penetration]\n");
            std::fprintf(output, "      contact=%u\n", contactIndex);
            std::fprintf(output, "      point=");
            printVector(output, contact.contactPoint);
            std::fprintf(output, "\n");
            std::fprintf(output, "      normal=");
            printVector(output, contact.contactNormal);
            std::fprintf(output, "\n");
            std::fprintf(output, "      depth=%.6f\n", (double)contact.penetration);
            std::fprintf(output, "      basis=");
            printMatrix(output, contactToWorld);
            std::fprintf(output, "\n");
            std::fprintf(output, "      desiredDeltaVelocity=%.6f\n", (double)desiredDeltaVelocity);
            std::fprintf(output, "      linearChange[0]=");
            printVector(output, linearChange[0]);
            std::fprintf(output, "\n");
            std::fprintf(output, "      linearChange[1]=");
            printVector(output, linearChange[1]);
            std::fprintf(output, "\n");
            std::fprintf(output, "      angularChange[0]=");
            printVector(output, angularChange[0]);
            std::fprintf(output, "\n");
            std::fprintf(output, "      angularChange[1]=");
            printVector(output, angularChange[1]);
            std::fprintf(output, "\n");
        }

        virtual void onPositionDepthUpdate(unsigned iteration,
                                           unsigned contactIndex,
                                           const cyclone::Contact &contact,
                                           cyclone::real penetrationDelta,
                                           cyclone::real newPenetration)
        {
            if (isSecondBoxGroundContact(contact) && penetrationDelta == (cyclone::real)0.0) return;
            if (!output) return;
            std::fprintf(output, "    [depthUpdate]\n");
            std::fprintf(output, "      contact=%u\n", contactIndex);
            std::fprintf(output, "      point=");
            printVector(output, contact.contactPoint);
            std::fprintf(output, "\n");
            std::fprintf(output, "      normal=");
            printVector(output, contact.contactNormal);
            std::fprintf(output, "\n");
            std::fprintf(output, "      depth=%.6f\n", (double)contact.penetration);
            std::fprintf(output, "      penetrationDelta=%.6f\n", (double)penetrationDelta);
            std::fprintf(output, "      newPenetration=%.6f\n", (double)newPenetration);
        }

        virtual void onVelocityIteration(unsigned iteration)
        {
            if (!output || !activeFrame) return;
            std::fprintf(output, "  [adjustVelocities] iteration=%u\n", iteration);
        }

        virtual void onVelocityResolution(unsigned iteration,
                                          unsigned contactIndex,
                                          const cyclone::Contact &contact,
                                          const cyclone::Matrix3 &contactToWorld,
                                          cyclone::real desiredDeltaVelocity,
                                          const cyclone::Vector3 impulse,
                                          const cyclone::Vector3 velocityChange[2],
                                          const cyclone::Vector3 rotationChange[2])
        {
            if (!output) return;
            std::fprintf(output, "    [velocityResolution]\n");
            std::fprintf(output, "      contact=%u\n", contactIndex);
            std::fprintf(output, "      point=");
            printVector(output, contact.contactPoint);
            std::fprintf(output, "\n");
            std::fprintf(output, "      normal=");
            printVector(output, contact.contactNormal);
            std::fprintf(output, "\n");
            std::fprintf(output, "      depth=%.6f\n", (double)contact.penetration);
            std::fprintf(output, "      basis=");
            printMatrix(output, contactToWorld);
            std::fprintf(output, "\n");
            std::fprintf(output, "      desiredDeltaVelocity=%.6f\n", (double)desiredDeltaVelocity);
            std::fprintf(output, "      impulse=");
            printVector(output, impulse);
            std::fprintf(output, "\n");
            std::fprintf(output, "      velocityChange[0]=");
            printVector(output, velocityChange[0]);
            std::fprintf(output, "\n");
            std::fprintf(output, "      velocityChange[1]=");
            printVector(output, velocityChange[1]);
            std::fprintf(output, "\n");
            std::fprintf(output, "      rotationChange[0]=");
            printVector(output, rotationChange[0]);
            std::fprintf(output, "\n");
            std::fprintf(output, "      rotationChange[1]=");
            printVector(output, rotationChange[1]);
            std::fprintf(output, "\n");
        }

        virtual void onDesiredVelocityUpdate(unsigned iteration,
                                             unsigned contactIndex,
                                             const cyclone::Contact &contact,
                                             const cyclone::Vector3 &contactVelocityDelta,
                                             cyclone::real oldDesiredDeltaVelocity,
                                             cyclone::real newDesiredDeltaVelocity)
        {
            if (!output) return;
            std::fprintf(output, "    [desiredVelocityUpdate]\n");
            std::fprintf(output, "      contact=%u\n", contactIndex);
            std::fprintf(output, "      point=");
            printVector(output, contact.contactPoint);
            std::fprintf(output, "\n");
            std::fprintf(output, "      normal=");
            printVector(output, contact.contactNormal);
            std::fprintf(output, "\n");
            std::fprintf(output, "      depth=%.6f\n", (double)contact.penetration);
            std::fprintf(output, "      contactVelocityDelta=");
            printVector(output, contactVelocityDelta);
            std::fprintf(output, "\n");
            std::fprintf(output, "      desiredDeltaVelocityOld=%.6f\n", (double)oldDesiredDeltaVelocity);
            std::fprintf(output, "      desiredDeltaVelocityNew=%.6f\n", (double)newDesiredDeltaVelocity);
        }
    };

    Box boxes[2];
    BoxesCollisionLogger collisionLogger;

    virtual void generateContacts();
    virtual void updateObjects(cyclone::real duration);
    virtual void reset();

public:
    BoxesDemo();
    virtual const char* getTitle();
    virtual void display();
    virtual void update();
};

BoxesDemo::BoxesDemo()
    : RigidBodyApplication(),
      collisionLogger(240)
{
    reset();
    pauseSimulation = false;
    collisionLogger.setTrackedBodies(boxes[0].body, boxes[1].body);
    resolver.setDebugListener(&collisionLogger);
}

const char* BoxesDemo::getTitle()
{
    return "Cyclone > Two Boxes Demo";
}

void BoxesDemo::generateContacts()
{
    cyclone::CollisionPlane plane;
    plane.direction = cyclone::Vector3(0, 1, 0);
    plane.offset = 0;

    cData.reset(maxContacts);
    cData.friction = (cyclone::real)0.9;
    cData.restitution = (cyclone::real)0.2;
    cData.tolerance = (cyclone::real)0.1;

    for (unsigned i = 0; i < 2; ++i)
    {
        if (!cData.hasMoreContacts()) return;
        cyclone::CollisionDetector::boxAndHalfSpace(boxes[i], plane, &cData);
    }

    if (!cData.hasMoreContacts()) return;
    cyclone::CollisionDetector::boxAndBox(boxes[0], boxes[1], &cData);
}

void BoxesDemo::updateObjects(cyclone::real duration)
{
    for (unsigned i = 0; i < 2; ++i)
    {
        boxes[i].body->integrate(duration);
        boxes[i].calculateInternals();
    }
}

void BoxesDemo::reset()
{
    const cyclone::Vector3 halfSize((cyclone::real)0.65, (cyclone::real)0.65, (cyclone::real)0.65);
    const cyclone::real mass = (cyclone::real)10.0;

    // Box #1 standing on the ground: center at y = 1.3 / 2.
    boxes[0].setState(cyclone::Vector3(0, (cyclone::real)0.65, 0), halfSize, mass);
    boxes[0].body->setAwake(false);
    // Box #2 above: center at y = 7.
    boxes[1].setState(cyclone::Vector3(0, (cyclone::real)7.0, 0), halfSize, mass);
    boxes[1].body->setAwake();

    cData.contactCount = 0;
}

void BoxesDemo::display()
{
    const static GLfloat lightPosition[] = {0.7f, 1.0f, 0.4f, 0.0f};

    RigidBodyApplication::display();

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_NORMALIZE);
    glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);

    boxes[0].render();
    boxes[1].render();

    glDisable(GL_NORMALIZE);
    glDisable(GL_LIGHTING);
    glDisable(GL_COLOR_MATERIAL);

    glColor3f(0.75f, 0.75f, 0.75f);
    for (unsigned i = 1; i < 20; ++i)
    {
        glBegin(GL_LINE_LOOP);
        for (unsigned j = 0; j < 32; ++j)
        {
            const float theta = 3.1415926f * j / 16.0f;
            glVertex3f(i * cosf(theta), 0.0f, i * sinf(theta));
        }
        glEnd();
    }
    glBegin(GL_LINES);
    glVertex3f(-20, 0, 0);
    glVertex3f(20, 0, 0);
    glVertex3f(0, 0, -20);
    glVertex3f(0, 0, 20);
    glEnd();

    char motionText[128];
    std::snprintf(
        motionText,
        sizeof(motionText),
        "Box 1 motion: %.4f\nBox 2 motion: %.4f",
        (double)boxes[0].body->getMotion(),
        (double)boxes[1].body->getMotion()
    );
    glColor3f(0.05f, 0.05f, 0.05f);
    renderText(10.0f, (float)height - 20.0f, motionText);

    RigidBodyApplication::drawDebug();
}

void BoxesDemo::update()
{
    float duration = 1.0/120;

    if (pauseSimulation)
    {
        Application::update();
        return;
    }
    else if (autoPauseSimulation)
    {
        pauseSimulation = true;
        autoPauseSimulation = false;
    }

    updateObjects(duration);
    generateContacts();

    bool boxesColliding = false;
    for (unsigned i = 0; i < cData.contactCount; ++i)
    {
        const cyclone::Contact &contact = cData.contactArray[i];
        const bool firstBodyMatch = contact.body[0] == boxes[0].body || contact.body[0] == boxes[1].body;
        const bool secondBodyMatch = contact.body[1] == boxes[0].body || contact.body[1] == boxes[1].body;
        if (contact.body[0] && contact.body[1] && firstBodyMatch && secondBodyMatch)
        {
            boxesColliding = true;
            break;
        }
    }

    collisionLogger.beginFrame(duration, boxesColliding);
    resolver.resolveContacts(
        cData.contactArray,
        cData.contactCount,
        duration
    );
    collisionLogger.endFrame();

    Application::update();
}

Application* getApplication()
{
    return new BoxesDemo();
}

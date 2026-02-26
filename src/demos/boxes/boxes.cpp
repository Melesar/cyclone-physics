/*
 * A simple two-box stacking demo.
 *
 * Part of the Cyclone physics system.
 */

#include <cyclone/cyclone.h>
#include <cstdio>
#include "../ogl_headers.h"
#include "../app.h"

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
        body->setAwake(true);
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
    Box boxes[2];

    virtual void generateContacts();
    virtual void updateObjects(cyclone::real duration);
    virtual void reset();

public:
    BoxesDemo();
    virtual const char* getTitle();
    virtual void display();
};

BoxesDemo::BoxesDemo()
    : RigidBodyApplication()
{
    reset();
    pauseSimulation = false;
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
    // Box #2 above: center at y = 7.
    boxes[1].setState(cyclone::Vector3(0, (cyclone::real)7.0, 0), halfSize, mass);

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

Application* getApplication()
{
    return new BoxesDemo();
}

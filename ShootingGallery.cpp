/*
 * The Shooting Gallery, by Mark Brosche, was made by adapting the BigBallistic demo.  
 *
 * Part of the Cyclone physics system.
 *
 * Copyright (c) Icosagon 2003. All Rights Reserved.
 *
 * This software is distributed under licence. Use of this software
 * implies agreement with all terms and conditions of the accompanying
 * software licence.
 */

#include <gl/glew.h>	// Used by lighting system and drawing text on the window?
#include <gl/glut.h>
#include <cyclone.h>
#include "app.h"
#include "timing.h"
#include "utility.h"	// Used to compute rotation matrices and print out numbers.

#include <stdio.h>
#include <sstream>		// Used to print variables to strings.
#include "ObjModel.h"
#include "PPMImage.h"


enum ShotType
{
    UNUSED = 0,
    PISTOL
};

/** The AmmoRound class stores the information for instantiating
and updating bullets, physics is applied when bullets are fired from the gun. */
class AmmoRound : public cyclone::CollisionSphere
{
public:
    ShotType type;
    unsigned startTime;
	cyclone::Vector3 velocityVecLocal = {0.0f, 0.0f, 20.0f}, velocityVecWorld;

    AmmoRound()
    {
        body = new cyclone::RigidBody;
    }

    ~AmmoRound()
    {
        delete body;
    }

    /** Draws the shot, excluding its shadow. */
    void render(cyclone::Vector3 gunEulerAngle, cyclone::Vector3 ammoCamOffset)
    {
        // Get the OpenGL transformationcyclone::Vector3 gunEuler, cyclone::Vector3 ammoOffsetWorld
		glDisable(GL_TEXTURE_2D);
		glColor3f(0.8f, 0.3f, 0.0f);
        GLfloat mat[16];
        body->getGLTransform(mat);
        glPushMatrix();
        glMultMatrixf(mat);
        glutSolidSphere(radius, 20, 20);
        glPopMatrix();
    }

    /** Sets the shot to a specific location. */
    void setState(ShotType shotType, cyclone::Vector3 position, cyclone::Vector3 angle)
    {		
		type = shotType;

        // Set the properties of the shot
        switch(type)
        {
        case PISTOL:
            body->setMass(1.50f);
			body->setPosition(position.x, position.y, position.z);
			body->setOrientation(1, 0, 0, 0);
			velocityVecWorld = computeRotatedVector(velocityVecLocal, angle);  // Derive the world velocity of the bullet from the angle of the gun
            body->setVelocity(velocityVecWorld.x, velocityVecWorld.y, velocityVecWorld.z);  
            body->setAcceleration(0.0f, -.50f, 0.0f);
            body->setDamping(0.99f, 0.8f);
            radius = 0.03f;
            break;
        }

        body->setCanSleep(false);
        body->setAwake();

        cyclone::Matrix3 tensor;
        cyclone::real coeff = 0.4f*body->getMass()*radius*radius;
        tensor.setInertiaTensorCoeffs(coeff,coeff,coeff);
        body->setInertiaTensor(tensor);

        // Set the data common to all particle types
        startTime = TimingData::get().lastFrameTimestamp;

        // Clear the force accumulators
        body->calculateDerivedData();
        calculateInternals();
    }
};

/** The Bullseye class stores the information for instantiating
and updating targets, physics is applied when collisions with bullets are detected. */
class Bullseye : public cyclone::CollisionBox
{
public:
	// Holds the OBJ model in memory.
	ObjModel bullseye;
	// ID for calling a list.
	GLuint bullseyeID;
	// Holds the hit status of a bullseye.
	bool hit = false;    
	
	Bullseye()
    {
        body = new cyclone::RigidBody;
    }

    ~Bullseye()
    {
        delete body;
    }

	/** Read the bullseye model into memory and create a display list*/
	void loadBullseyeModel()
	{
		bullseye.ReadFile("Models/target.obj");
		bullseyeID = glGenLists(1);
		glNewList(bullseyeID, GL_COMPILE);
		bullseye.Draw();
		glEndList();		
	}

	/** Draws the bullseye, excluding its shadow. */
    void render()
    {     
		// Get the OpenGL transformation
        GLfloat mat[16];
        body->getGLTransform(mat);

        glPushMatrix();
        glMultMatrixf(mat);
		// The model doesn't really need scaling but the rigid-body must be sized according to the model dimensions!
		glScalef(halfSize.x /1.2, halfSize.y /3.0, halfSize.z / 1.0); 		
		glCallList(bullseyeID);		
		glPopMatrix();
    }

    /** Sets the bullseye to a specific location. */
    void setState(cyclone::real x, cyclone::real z)
    {
        body->setPosition(x, 2.9, z);
		body->setOrientation(0,0,1,0);
		body->setVelocity(5, 0, 0);
        body->setRotation(cyclone::Vector3(0,0,0));
        halfSize = cyclone::Vector3(1.2,3,1); // Half-dimensions of the target model.

        cyclone::real mass = halfSize.x * halfSize.y * halfSize.z * .10f;
        body->setMass(mass);

        cyclone::Matrix3 tensor;
        tensor.setBlockInertiaTensor(halfSize, mass);
        body->setInertiaTensor(tensor);

        body->setLinearDamping(0.95f);
        body->setAngularDamping(0.8f);
        body->clearAccumulators();
        body->setAcceleration(0,0,0);

        body->setCanSleep(false);
        body->setAwake();

        body->calculateDerivedData();
        calculateInternals();
    }
};

/** The Gun class stores the information for instantiating 
and updating a Gun model, physics is not applied. */
class Gun : public cyclone::CollisionBox
{
public:
	Gun()
	{
		body = new cyclone::RigidBody;
	}

	~Gun()
	{
		delete body;
	}

	ObjModel gun;
	GLuint gunID;

	/** Reads the OBJ model file into memory and creates a display list.*/
	void loadGunModel()
	{
		gun.ReadFile("Models/revolver.obj");
		gunID = glGenLists(1);
		glNewList(gunID, GL_COMPILE);
		gun.Draw();
		glEndList();
	}

	/** Draws the model without a shadow. */
	void render(cyclone::Vector3 gunEulerAngle, cyclone::Vector3 gunCamOffset)
	{
		// Get the OpenGL transformation
		GLfloat mat[16];
		body->getGLTransform(mat);

		glPushMatrix();
		glMultMatrixf(mat);
		glTranslatef(0, gunCamOffset.y, 0);
		glRotatef(gunEulerAngle.y, 0, 1, 0);
		glRotatef(gunEulerAngle.x, 1, 0, 0);	
		glTranslatef(gunCamOffset.x, 0, gunCamOffset.z);
		glScalef(1.0f, 1.0f, 1.0f);
		glCallList(gunID);
		glPopMatrix();
	}

	/*Sets the location of the gun*/
	void setState(cyclone::Vector3 position)
	{
		body->setPosition(position.x, position.y, position.z);
		body->setOrientation(1, 0, 0, 0);
		body->setVelocity(0, 0, 0);
		body->setRotation(cyclone::Vector3(0, 0, 0));
		halfSize = cyclone::Vector3(1, 1, 1);

		cyclone::real mass = halfSize.x * halfSize.y * halfSize.z * 10.0f;
		body->setMass(mass);

		cyclone::Matrix3 tensor;
		tensor.setBlockInertiaTensor(halfSize, mass);
		body->setInertiaTensor(tensor);

		body->setLinearDamping(0.95f);
		body->setAngularDamping(0.8f);
		body->clearAccumulators();
		body->setAcceleration(0, 0, 0);

		body->setCanSleep(false);
		body->setAwake();

		body->calculateDerivedData();
		calculateInternals();
	}
};

/** The main demo class definition. */
class ShootingGallery : public RigidBodyApplication
{
	/** Holds the maximum number of  rounds that can be fired. */
    const static unsigned ammoRounds = 6;

	/** Mutable count of ammunition remaining in the weapon.*/
	int ammoCount = ammoRounds;

    /** Holds the particle data. */
    AmmoRound ammo[ammoRounds];
	
	/** Holds the number of guns in the simulation. */
	const static unsigned guns = 1;

	/** Holds the gun data. */
	Gun revolver[guns];

    /** Holds the number of bullseye targets in the simulation. */
    const static unsigned bullseyes = 10;

    /** Holds the bullseye data. */
    Bullseye bullseyeData[bullseyes];

    /** Holds the current shot type. */
    ShotType currentShotType;

    /** Resets the position of all the boxes and primes the explosion. */
    virtual void reset();

    /** Build the contacts for the current situation. */
    virtual void generateContacts();

    /** Processes the objects in the simulation forward in time. */
    virtual void updateObjects(cyclone::real duration);

    /** Dispatches a round. */
    void fire();

	/** Records the number of targets hit. */
	int score = 0;
	int targetsRemaining = bullseyes;

	/** Store the OBJ file for the static scenery. */
	ObjModel gallery;
	int galleryID;
	
	/** Read in and create a call list for the static scenery. */
	void loadScene();

	/** Draw the static scenery. */
	void drawScene();

	/** Hold offset vectors of Camera, Ammo from the gun in Local and Worldspace. */
	cyclone::Vector3
		cameraOffsetLocal = { 0.0f, 4.5f, -3.0f },		// offset from world point {0,0,0}
		cameraOffsetWorld = { cameraOffsetLocal.x, cameraOffsetLocal.y, cameraOffsetLocal.z },
		aimOffsetLocal = { 0.0f, 4.5f, 50.0f },			// offset from world point {0,0,0}
		aimOffsetWorld = { aimOffsetLocal.x, aimOffsetLocal.y, aimOffsetLocal.z },
		gunOffsetLocal = { -0.33f, 4.25f, -1.5f },		// offset from world point {0,0,0}
		gunOffsetWorld = { gunOffsetLocal.x, gunOffsetLocal.y, gunOffsetLocal.z },
		ammoOffsetLocal = { 0.33f, 0.25f, -2.0f },		// offset from world camera point
		ammoOffsetWorld = { ammoOffsetLocal.x, ammoOffsetLocal.y, ammoOffsetLocal.z },
		gunEuler = { 0.0f, 0.0f, 0.0f };
	
public:
    /** Creates a new demo object. */
    ShootingGallery();

    /** Returns the window title for the demo. */
    virtual const char* getTitle();

    /** Sets up the rendering. */
    virtual void initGraphics();
    
    /** Display world. */
    virtual void display();

    /** Handle a keypress. */
    virtual void key(unsigned char key);

	/** Handle a special keypress (Arrow keys for this app). */
	virtual void specialKey(int specialKey);
};

// Method definitions
ShootingGallery::ShootingGallery():RigidBodyApplication(),
currentShotType(PISTOL)
{
    pauseSimulation = false;
    reset();
}

void ShootingGallery::loadScene()
{
	glEnable(GL_COLOR_MATERIAL);
	glDisable(GL_LIGHTING);
	gallery.ReadFile("Models/gallery.obj");
	galleryID = glGenLists(1);
	glNewList(galleryID, GL_COMPILE);	
	gallery.Draw();
	glEndList();
	glDisable(GL_COLOR_MATERIAL);
	glEnable(GL_LIGHTING);
}

void ShootingGallery::drawScene()
{
	glPushMatrix();
	glTranslatef(0, 0, 0);
	glCallList(galleryID);
	glPopMatrix();
}

void ShootingGallery::initGraphics()
{
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	GLfloat lightAmbient[] = {0.3f, 0.3f, 0.3f, 1.0f};
    GLfloat lightDiffuse[] = {0.9f, 0.95f, 1.0f, 1.0f};
	GLfloat lightSpecular[] = { 1.0, 1.0, 1.0, 1 };

	GLfloat light_position[] = { 0.0, 50.0, 0.0, 0.0 };
	
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);
	glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, lightSpecular);

	glLightModeli(GL_LIGHT_MODEL_COLOR_CONTROL, GL_SEPARATE_SPECULAR_COLOR);
	glLightModelf(GL_LIGHT_MODEL_LOCAL_VIEWER, 1);

	glShadeModel(GL_SMOOTH);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_NORMALIZE);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	ShootingGallery::loadScene();

	for (Gun *gun = revolver; gun < revolver + guns; gun++)
	{
		gun->loadGunModel();
	}
	for (Bullseye *bullseye = bullseyeData; bullseye < bullseyeData + bullseyes; bullseye++)
	{
		bullseye->loadBullseyeModel();
	}

    Application::initGraphics();
}

void ShootingGallery::reset()
{
	// Reset all vars to initial values.
	cameraOffsetWorld = { cameraOffsetLocal.x, cameraOffsetLocal.y, cameraOffsetLocal.z };
	aimOffsetWorld = { aimOffsetLocal.x, aimOffsetLocal.y, aimOffsetLocal.z };
	gunOffsetWorld = { gunOffsetLocal.x, gunOffsetLocal.y, gunOffsetLocal.z };
	ammoOffsetWorld = { ammoOffsetLocal.x, ammoOffsetLocal.y, ammoOffsetLocal.z };
	gunEuler = { 0, 0, 0 };
	score = 0;	
	targetsRemaining = bullseyes;

	// Make all shots unused
    for (AmmoRound *shot = ammo; shot < ammo+ammoRounds; shot++)
    {
        shot->type = UNUSED;
    }

    // Initialise the bullseye
    cyclone::real x = -40.0f, z = 9.5;
	int perRow=0;
    for (Bullseye *bullseye = bullseyeData; bullseye < bullseyeData+bullseyes; bullseye++)
    {
		if (perRow < 5)		// Spawn the first 5 targets in the front row.
		{
			bullseye->setState(x, z);
			x += 10.0f;
			perRow++;
		}
		else				// Spawn the rest of the targets in the back row.
		{
			z = 19.5;
			bullseye->setState(x, z);
			x += 15.0f;
		}
		bullseye->hit = FALSE;
    }

	// Initialize the gun
	for (Gun *gun = revolver; gun < revolver + guns; gun++)
	{
		gun->setState(cameraOffsetWorld);
	}
}

const char* ShootingGallery::getTitle()
{
    return "Cyclone > Assignment 2: Shooting Gallery";
}

void ShootingGallery::fire()
{
    // Find the first available round.
    AmmoRound *shot;
    for (shot = ammo; shot < ammo+ammoRounds; shot++)
    {
        if (shot->type == UNUSED) break;
    }

    // If we didn't find a round, then exit - we can't fire.
    if (shot >= ammo+ammoRounds) return;

    // Set the shot
	shot->setState(currentShotType, cameraOffsetWorld - ammoOffsetWorld, gunEuler);
	if (ammoCount > 0) ammoCount--;
}

void ShootingGallery::updateObjects(cyclone::real duration)
{
    // Update the physics of each particle in turn
    for (AmmoRound *shot = ammo; shot < ammo+ammoRounds; shot++)
    {
        if (shot->type != UNUSED)
        {
            // Run the physics
            shot->body->integrate(duration);
            shot->calculateInternals();

            // Check if the particle is now invalid
            if (shot->body->getPosition().y < 0.0f ||
                shot->startTime+5000 < TimingData::get().lastFrameTimestamp ||
                shot->body->getPosition().z > 200.0f)
            {
                // We simply set the shot type to be unused, so the
                // memory it occupies can be reused by another shot.
                shot->type = UNUSED;
				if (ammoCount <= 0)	ammoCount = 6;
            }
        }
    }

    // Update the bullseyes
	for (Bullseye *bullseye = bullseyeData; bullseye < bullseyeData + bullseyes; bullseye++)
	{
		// Run the physics
		bullseye->body->integrate(duration);
		bullseye->calculateInternals();

		// Oscillate the bullseyes
		if (bullseye->body->getPosition().x <= (- 15.0f))
			bullseye->body->setVelocity(5.0f, 0.0f, 0.0f);
		else if (bullseye->body->getPosition().x >= (15.0f))
			bullseye->body->setVelocity(-5.0f, 0.0f, 0.0f);
    }
}

void ShootingGallery::display()
{
    // Clear the viewport and set the camera direction.
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Draw the static environment.
	ShootingGallery::drawScene();

    // Render each bullet particle in turn.
    for (AmmoRound *shot = ammo; shot < ammo+ammoRounds; shot++)
    {
        if (shot->type != UNUSED)
        {
            shot->render(gunEuler, ammoOffsetWorld - cameraOffsetLocal);
        }
    }

    // Render gun and target models.
	for (Gun *gun = revolver; gun < revolver+guns; gun++)
	{
		gun->render(gunEuler, gunOffsetWorld-cameraOffsetLocal);
	}

    for (Bullseye *bullseye = bullseyeData; bullseye < bullseyeData+bullseyes; bullseye++)
    {
		bullseye->render();
    }

	glDisable(GL_LIGHTING);
	glDisable(GL_TEXTURE_2D);
	
	// Display the game instructions.
	glColor3f(0.0, 0.0, 0.0);	renderText(10.0f, height - 24.0, "Space: Fire \nWASD/Up/Down: Aim \nR: Reset \nEsc: Quit");
	glColor3f(1.0, 1.0, 1.0);	renderText(9.0f, height - 23.0, "Space: Fire \nWASD/Up/Down: Aim \nR: Reset \nEsc: Quit");
	
	// Display the score.
	glColor3f(0.0, 0.0, 0.0); renderText(width*0.45, height - 72.0, "Score: ");
	glColor3f(1.0, 1.0, 1.0); renderText(width*0.4495, height - 71.0, "Score: ");
	stringstream ss1, ss2, ss3;
	ss1 << score;
	printLargeString(ss1.str());

	// Display the number of targets left.
	glColor3f(0.0, 0.0, 0.0); renderText(width*0.45, height - 96.0, "Targets Remaining: ");
	glColor3f(1.0, 1.0, 1.0); renderText(width*0.4495, height - 95.0, "Targets Remaining: ");
	ss2 << targetsRemaining;
	printLargeString(ss2.str());

	// Display Ammo count
	glColor3f(0.0, 0.0, 0.0); renderText(width*0.90, height - 24.0, "Ammo: ");
	glColor3f(1.0, 1.0, 1.0); renderText(width*0.8995, height - 23.0, "Ammo: ");
	ss3 << ammoCount;
	printLargeString(ss3.str());

	// Display a warning message if player aims outside acceptable target area.
	if (gunEuler.x <= -30 || gunEuler.y <= -45 || gunEuler.y >= 45)
	{
		glColor3f(1.0, 1.0, 0.0); renderText(width * 0.425, height -150.0, "Please aim at the targets only!");
		glColor3f(1.0, 0.0, 0.0); renderText(width * 0.4249, height - 149.0, "Please aim at the targets only!");
	}

	// Display a Win message
	if (score == 10)
	{
		glColor3f(0.0, 0.0, 0.0); renderText(width * 0.4755, height - 151.0, "You Win!");
		glColor3f(1.0, 0.0, 0.0); renderText(width * 0.475, height - 150.0, "You Win!");
		glColor3f(1.0, 0.5, 0.0); renderText(width * 0.4749, height - 149.0, "You Win!");
		glColor3f(1.0, 1.0, 0.0); renderText(width * 0.4745, height - 148.0, "You Win!");
		glColor3f(0.0, 1.0, 0.0); renderText(width * 0.474, height - 147.0, "You Win!");
		glColor3f(0.0, 1.0, 1.0); renderText(width * 0.4735, height - 146.0, "You Win!");
		glColor3f(0.0, 0.0, 1.0); renderText(width * 0.473, height - 145.0, "You Win!");
		glColor3f(1.0, 0.0, 1.0); renderText(width * 0.4725, height - 144.0, "You Win!");
	}

	// Configure the Game Camera to look where you are aiming.
	glLoadIdentity();
	gluLookAt(
		cameraOffsetLocal.x,	// Does not change. 
		cameraOffsetWorld.y,
		cameraOffsetLocal.z,	// Does not change. 
		aimOffsetWorld.x,
		aimOffsetWorld.y,
		aimOffsetWorld.z,
		0.0, 1.0, 0.0);	
}

void ShootingGallery::generateContacts()
{
    // Create the ground plane data
    cyclone::CollisionPlane plane;
    plane.direction = cyclone::Vector3(0,1,0);
    plane.offset = -2.0f; // Collision plane lowered so the targets have a chance to fall down before removing

    // Set up the collision data structure
    cData.reset(maxContacts);
    cData.friction = (cyclone::real)0.9;
    cData.restitution = (cyclone::real)0.1;
    cData.tolerance = (cyclone::real)0.01;


    // Check ground plane collisions
    for (Bullseye *bullseye = bullseyeData; bullseye < bullseyeData+bullseyes; bullseye++)
    {
        if (!cData.hasMoreContacts()) return;
		if (cyclone::CollisionDetector::boxAndHalfSpace(*bullseye, plane, &cData))
		{
			// Shrink the bullseye to zero after it falls and increment the score count and decrement the target count.
			bullseye->halfSize.z = 0;
			bullseye->halfSize.y = 0;
			bullseye->halfSize.x = 0;	
			bullseye->body->setAwake(false);


			// Ensure the score only increments once for each bullseye
			if (bullseye->hit == false) 
			{
				score++;
				targetsRemaining--;
				bullseye->hit = true;
			}			
		}

        // Check for collisions with each shot
        for (AmmoRound *shot = ammo; shot < ammo+ammoRounds; shot++)
        {			
            if (shot->type != UNUSED)
            {
                
				if (!cData.hasMoreContacts()) return;

                // When we get a collision, remove the shot and the bullseye
                if (cyclone::CollisionDetector::boxAndSphere(*bullseye, *shot, &cData))
                {
                    shot->type = UNUSED;
					if (ammoCount <= 0) ammoCount = 6;
					// Stop the target in its track when hit.
					bullseye->body->setVelocity(0, 0, 0);
					// Allow gravity to act on the target when hit.
					bullseye->body->setAcceleration(0,-10.0f, 0);					 
					// Add force of bullet impact on the target where it is hit.
					bullseye->body->addForceAtBodyPoint(shot->body->getVelocity(), shot->body->getPosition());
                }
            }
        }
    }
    // NB We aren't checking box-box collisions.
}

/** This method controls the effect of standard keys. */
void ShootingGallery::key(unsigned char key)
{
    switch(key)
    {
	case 'w': case 'W':		/*pitch gun up, (increases gluLookat target y value)*/		
		gunEuler.x-=.5;
		if (gunEuler.x > -70)
		{
			aimOffsetWorld = computeRotatedVector(aimOffsetLocal, gunEuler);
			ammoOffsetWorld = computeRotatedVector(ammoOffsetLocal, gunEuler);
		}
		else if (gunEuler.x < -70) gunEuler.x = -70;
		break;

	case 's': case 'S':		/*pitch gun down (decreases gluLookat target y value)*/
		gunEuler.x+=.5;
		if (gunEuler.x < 70)
		{
			aimOffsetWorld = computeRotatedVector(aimOffsetLocal, gunEuler);
			ammoOffsetWorld = computeRotatedVector(ammoOffsetLocal, gunEuler);
		}
		else if (gunEuler.x > 70) gunEuler.x = 70;
		break;

	case 'a': case 'A':		/*yaw gun to the left (decreases gluLookat target x value)*/
		gunEuler.y+=.5;
		if (gunEuler.y < 90)
		{
			aimOffsetWorld = computeRotatedVector(aimOffsetLocal, gunEuler);
			ammoOffsetWorld = computeRotatedVector(ammoOffsetLocal, gunEuler);
		}
		else if (gunEuler.y > 90) gunEuler.y = 90;
		break;

	case 'd': case 'D':		/*yaw gun to the right (increases gluLookat target x value)*/
		gunEuler.y-=.5;
		if (gunEuler.y > -90)
		{
			aimOffsetWorld = computeRotatedVector(aimOffsetLocal, gunEuler);
			ammoOffsetWorld = computeRotatedVector(ammoOffsetLocal, gunEuler);
		}
		else if (gunEuler.y < -90) gunEuler.y = -90;		
		break;

	case ' ': fire(); break;

    case 'r': case 'R': reset(); break;

	case 27: exit(0); break;
    }
 }

/** This method controls the effect of the arrow keys. */
void ShootingGallery::specialKey(int specialKey)
{
	if (specialKey == GLUT_KEY_UP)		// move gun vertically up (increases gluLookAt eye y and target y value)
	{
		for (Gun *gun = revolver; gun < revolver + guns; gun++)
		{
			cameraOffsetWorld.y += 0.1f;
			aimOffsetWorld.y += 0.1f;
			gunOffsetWorld.y += 0.1f;
		}
	}

	if (specialKey == GLUT_KEY_DOWN)	// move gun vertically down (decreases gluLookAt eye y and target y value)
	{
		for (Gun *gun = revolver; gun < revolver + guns; gun++)
		{
			cameraOffsetWorld.y -= 0.1f;
			aimOffsetWorld.y -= 0.1f;
			gunOffsetWorld.y -= 0.1f;
		}
	}
}

/**
 * Called by the common demo framework to create an application
 * object (with new) and return a pointer.
 */
Application* getApplication()
{
    return new ShootingGallery();
}


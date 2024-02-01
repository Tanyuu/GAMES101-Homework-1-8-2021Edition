#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.
        Vector2D t = (end - start)/(num_nodes-1);
        for(int i=0;i<num_nodes;i++)
        {
            masses.push_back(new Mass(start + i*t, node_mass, false));
        }
        for(int i=1;i<num_nodes;i++)
        {
            springs.push_back(new Spring(masses[i-1], masses[i],k));
        }
//        Comment-in this part when you implement the constructor
        for (auto &i : pinned_nodes) {
            masses[i]->pinned = true;
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            Vector2D ab = s->m2->position - s->m1->position;
            Vector2D f_to_b = s->k * ab / ab.norm() * (ab.norm() - s->rest_length);
            s->m2->forces -= f_to_b;
            s->m1->forces += f_to_b;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                m->forces+= m->mass*gravity;
                // TODO (Part 2): Add global damping
                m->forces+= - 0.005 * m->velocity;

                Vector2D a = m->forces / m->mass;
                // std::cout<<m->forces<<' '<<m->mass<<std::endl;
                // getchar();
                static bool Euler = false;

                if(Euler) // Explicit Euler // NOT WORK !! FLY THE ROPES
                {
                    m->position += delta_t * m->velocity;
                    m->velocity += delta_t * a;
                }
                else // Semi-implicit Euler
                {
                    m->velocity += delta_t * a;
                    m->position += delta_t * m->velocity;
                }
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
        
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            Vector2D ab = s->m2->position - s->m1->position;
            Vector2D f_to_b = s->k * ab / ab.norm() * (ab.norm() - s->rest_length);
            s->m2->forces -= f_to_b;
            s->m1->forces += f_to_b;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                m->forces+= m->mass*gravity;
                Vector2D a = m->forces / m->mass;

                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                // TODO (Part 4): Add global Verlet damping
                m->position = m->position +  (1 - 0.00005) * (m->position - m->last_position) + a * delta_t * delta_t;\
                m->last_position = temp_position;
            }
            m->forces = Vector2D(0,0);
        }
    }
}
